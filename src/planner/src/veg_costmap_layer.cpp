#include "planner/veg_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "planner_msgs/srv/update_cost.hpp"
#include "planner_msgs/srv/get_transforms.hpp"

/**
 * @file veg_costmap_layer.cpp
 * @brief Implementation of a costmap layer for dynamic vegetation environments.
 * Custom nav2 costmap: https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html
 *
 * Costmap values are 0-255:
 *    0 = FREE_SPACE
 *    1-252 = Increasing cost values
 *    253 = INSCRIBED_INFLATED_OBSTACLE
 *    254 = LETHAL_OBSTACLE
 *    255 = NO_INFORMATION
 */
namespace veg_costmap
{
    /**
     * @brief VegCostmapLayer constructor
     */
    VegCostmapLayer::VegCostmapLayer() : gen(std::random_device()()) // Seed random generator
    {
    };

    /**
     * onInitialize()
     * @brief Method is called at the end of plugin initialization. There is usually
     * declarations of ROS parameters.
     * This is where any required initialization should occur.
     */
    void VegCostmapLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Failed to lock node");

        RCLCPP_INFO(node->get_logger(), "Initializing VegCostmapLayer...");

        // Declare parameters - using Layer's inherited methods
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("world_tf_service", rclcpp::ParameterValue("/world/get_tf"));
        declareParameter("update_topic", rclcpp::ParameterValue("/veg_costmap/update"));
        declareParameter("updated_topic", rclcpp::ParameterValue("/veg_costmap/updated"));
        declareParameter("costmap_topic", rclcpp::ParameterValue("/veg_costmap"));
        declareParameter("obstacle_range", rclcpp::ParameterValue(5.0));
        declareParameter("obstacle_radius", rclcpp::ParameterValue(0.5));
        declareParameter("lethal_cost", rclcpp::ParameterValue(254));
        declareParameter("use_gradient_costs", rclcpp::ParameterValue(true));
        declareParameter("gradient_factor", rclcpp::ParameterValue(0.8));
        declareParameter("map_width", rclcpp::ParameterValue(1000));
        declareParameter("map_height", rclcpp::ParameterValue(1000));
        declareParameter("map_resolution", rclcpp::ParameterValue(0.05));
        declareParameter("map_origin_x", rclcpp::ParameterValue(-25.0));
        declareParameter("map_origin_y", rclcpp::ParameterValue(-25.0));

        // Get parameters
        node->get_parameter(name_ + ".enabled", enabled_);
        node->get_parameter(name_ + ".world_tf_service", world_tf_service_);
        node->get_parameter(name_ + ".update_topic", update_topic_);
        node->get_parameter(name_ + ".updated_topic", updated_topic_);
        node->get_parameter(name_ + ".costmap_topic", costmap_pub_topic_);
        node->get_parameter(name_ + ".obstacle_range", obstacle_range_);
        node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
        node->get_parameter(name_ + ".lethal_cost", lethal_cost_);
        node->get_parameter(name_ + ".use_gradient_costs", use_gradient_costs_);
        node->get_parameter(name_ + ".gradient_factor", gradient_factor_);
        node->get_parameter(name_ + ".map_width", map_width_);
        node->get_parameter(name_ + ".map_height", map_height_);
        node->get_parameter(name_ + ".map_resolution", map_resolution_);
        node->get_parameter(name_ + ".map_origin_x", map_origin_x_);
        node->get_parameter(name_ + ".map_origin_y", map_origin_y_);

        global_frame_ = layered_costmap_->getGlobalFrameID();

        // Publisher to trigger replanning when costmap gets updated
        replan_pub_ = node->create_publisher<std_msgs::msg::Empty>(
            updated_topic_, 1);

        // Service for updating costmap values
        update_cost_srv_ = node->create_service<planner_msgs::srv::UpdateCost>(
            update_topic_,
            std::bind(&VegCostmapLayer::updateCostCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Publisher for visualizing the costmap
        costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
            costmap_pub_topic_, 1);
        // Create a timer to periodically publish the costmap
        costmap_pub_timer_ = node->create_wall_timer(
            std::chrono::milliseconds(500), // Period in ms
            std::bind(&VegCostmapLayer::publishCostmapCallback, this));

        // Init obstacle grid 2D to map size
        std::vector<std::vector<ObstaclePoint>> obstacle_grid_(
            static_cast<int>(map_width_),
            std::vector<ObstaclePoint>(static_cast<int>(map_height_)));

        // Init the obstacle_database_ unordered_map with prior knowledge
        std::unordered_map<std::string, ObstacleData> obstacle_database_ = veg_costmap::defaults::SAVED_OBSTACLE_DATABASE;

        // Get and set world obstacles
        VegCostmapLayer::getWorldTransforms();

        RCLCPP_INFO(node->get_logger(), "VegCostmapLayer initialized");
    }

    /**
     * @brief Update the cost value in the costmap layer and data structures
     * @param costmap The costmap to update
     * @param mx The x coordinate in the costmap
     * @param my The y coordinate in the costmap
     * @param cost The cost value to set
     * @return True if the cost was updated successfully
     */
    bool VegCostmapLayer::updateCostValue_(nav2_costmap_2d::Costmap2D *costmap, unsigned int mx, unsigned int my, unsigned char cost)
    {
        auto node = node_.lock();
        if (!node)
        {
            return false;
        }

        // Check if the cell is within the map bounds
        if (mx < 0 || mx >= map_width_ || my < 0 || my >= map_height_)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(),
                "Cannot update cost at (%u, %u): out of bounds",
                mx, my);
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // Unknown obstacle name
        std::string obstacle_name = "";

        // Calculate the bounds of the obstacle
        unsigned int radius_cells = std::max(1u, static_cast<unsigned int>(obstacle_radius_ / map_resolution_));

        for (int i = -static_cast<int>(radius_cells); i <= static_cast<int>(radius_cells); i++)
        {
            for (int j = -static_cast<int>(radius_cells); j <= static_cast<int>(radius_cells); j++)
            {
                double distance_sq = i * i + j * j;
                double radius_sq = radius_cells * radius_cells;

                // Skip cells outside the circle
                if (distance_sq > radius_sq)
                {
                    continue;
                }

                unsigned int cur_mx = mx + i;
                unsigned int cur_my = my + j;

                // Check if cell is within map bounds
                if (cur_mx < 0 || cur_mx >= costmap->getSizeInCellsX() ||
                    cur_my < 0 || cur_my >= costmap->getSizeInCellsY())
                {
                    continue;
                }

                ObstaclePoint &obstacle = obstacle_grid_[cur_mx][cur_my];
                // If obstacle has name, set to obstacle_name
                if (!obstacle.name.empty())
                {
                    obstacle_name = obstacle.name;
                }

                unsigned char new_cost = cost; // Default fixed cost
                // Apply gradient cost if enabled, otherwise use fixed cost
                if (use_gradient_costs_)
                {
                    // Gradient cost
                    // Calculate cost based on distance from center (higher at center, lower at edges)
                    double distance_factor = 1.0 - (std::sqrt(distance_sq) / radius_cells);
                    new_cost = static_cast<unsigned char>(
                        cost * (distance_factor * gradient_factor_ + (1.0 - gradient_factor_)));
                }
                // Bound cost to lethal cost
                new_cost = std::min(new_cost, static_cast<unsigned char>(lethal_cost_));
                // Update the costmap layer with the new cost
                costmap->setCost(cur_mx, cur_my, new_cost);
                // Update the obstacle grid with the code
                obstacle.cost = cost;
                obstacle.cost_stddev = 0; // Mark as already sampled!
            }
        }

        // If obstacle has name, update the obstacle_database_ with the new cost
        if (!obstacle_name.empty())
        {
            ObstacleData &obstacle_data = obstacle_database_[obstacle_name];
            if (obstacle_data.cost == 0)
            {
                // No prior data, set initial cost and stddev
                obstacle_data.cost = cost;
            }
            else
            {
                // Combine the new normal dist with the prior normal dist
                bayesian_update_gaussian(
                    // Old prior knowledge
                    obstacle_data.cost, obstacle_data.cost_stddev,
                    // New sample, assume twice as accurate as prior
                    cost, obstacle_data.cost_stddev / 2.0,
                    // Output updated mean and stddev
                    &obstacle_data.cost, &obstacle_data.cost_stddev);
            }
            obstacle_data.cost = cost;
            obstacle_data.cost_stddev = 0; // Mark as already sampled!

            // UPDATE similar obstacles
            // Iterate through all the other similar obstacles and update their cost
            for (auto &obstacle : obstacle_data.others)
            {
                ObstaclePoint &obstacle_point = obstacle_grid_[mx][my];
                // If stddev is not 0 (ie. not sampled), update cost from a normal distribution to account for uncertainty
                if (obstacle_point.cost_stddev != 0)
                {
                    std::normal_distribution<double> normal_dist(obstacle_data.cost, obstacle_data.cost_stddev);
                    // Sample from normal distribution
                    obstacle_point.cost = normal_dist(gen);
                }
            }
        }

        costmap_updated_ = true;

        // Publish notification that costmap was updated
        auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
        replan_pub_->publish(std::move(msg_empty));

        return true;
    }

    /**
     * @brief Service call to update cost of a specific point in the costmap
     * @param request The service request
     * @param response The service response
     */
    void VegCostmapLayer::updateCostCallback(
        const std::shared_ptr<planner_msgs::srv::UpdateCost::Request> request,
        std::shared_ptr<planner_msgs::srv::UpdateCost::Response> response)
    {
        auto node = node_.lock();
        if (!node)
        {
            RCLCPP_ERROR(node->get_logger(), "Cannot update cost: node is not available");
            response->success = false;
            response->message = "Node is not available";
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // Get the index and cost from the request
        float world_x = request->x;
        float world_y = request->y;
        unsigned char cost = request->cost; // unsigned char = uint8_t (0-255)
        RCLCPP_INFO(
            node->get_logger(),
            "Updating cost at (%.2f, %.2f) to %d",
            world_x, world_y, static_cast<int>(cost));

        // Convert to map coordinates
        unsigned int map_x, map_y;
        if (!layered_costmap_->getCostmap()->worldToMap(world_x, world_y, map_x, map_y))
        {
            response->success = false;
            response->message = "Coordinates out of map bounds";
            return;
        }

        VegCostmapLayer::updateCostValue_(layered_costmap_->getCostmap(), map_x, map_y, cost);

        // Set response
        response->success = true;
        response->message = "Cost updated successfully";
    }

    /**
     * updateBounds()
     * @brief Method is called to ask the plugin: which area of costmap layer it needs to
     * update.
     * @param inputs robot position and orientation
     * @param outputs pointers to window bounds
     * These bounds are used for performance reasons: to update the area inside
     * the window where new info is available, avoiding updates of the whole
     * costmap on every iteration.
     */
    void VegCostmapLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        std::lock_guard<std::mutex> lock(mutex_);

        if (costmap_updated_)
        {
            // Option 1: Update the entire map when updates affect many areas
            // *min_x = layered_costmap_->getCostmap()->getOriginX();
            // *min_y = layered_costmap_->getCostmap()->getOriginY();
            // *max_x = *min_x + layered_costmap_->getCostmap()->getSizeInMetersX();
            // *max_y = *min_y + layered_costmap_->getCostmap()->getSizeInMetersY();

            // Option 2: Loop through obstacle_grid_ to find bounds of changed areas
            // This is more efficient if changes are localized
            bool found_update = false;
            double update_min_x = std::numeric_limits<double>::max();
            double update_min_y = std::numeric_limits<double>::max();
            double update_max_x = std::numeric_limits<double>::lowest();
            double update_max_y = std::numeric_limits<double>::lowest();

            for (size_t i = 0; i < obstacle_grid_.size(); i++)
            {
                for (size_t j = 0; j < obstacle_grid_[i].size(); j++)
                {
                    const auto &obstacle = obstacle_grid_[i][j];
                    if (obstacle.cost > 0)
                    { // Only consider cells with obstacles
                        double wx, wy;
                        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);

                        update_min_x = std::min(update_min_x, wx - obstacle_radius_);
                        update_min_y = std::min(update_min_y, wy - obstacle_radius_);
                        update_max_x = std::max(update_max_x, wx + obstacle_radius_);
                        update_max_y = std::max(update_max_y, wy + obstacle_radius_);
                        found_update = true;
                    }
                }
            }

            if (found_update)
            {
                *min_x = std::min(*min_x, update_min_x);
                *min_y = std::min(*min_y, update_min_y);
                *max_x = std::max(*max_x, update_max_x);
                *max_y = std::max(*max_y, update_max_y);
            }

            costmap_updated_ = false;
        }
    }

    /**
     * updateCosts()
     * Method is called each time when costmap re-calculation is required. It
     * updates the costmap layer only within its bounds window.
     * @param inputs calculation window bounds
     * @param outputs reference to a resulting costmap master_grid
     * The Layer class provides the plugin with an internal costmap, costmap_,
     * for updates. The master_grid should be updated with values within the
     * window bounds using one of the following update methods:
     * updateWithAddition(), updateWithMax(), updateWithOverwrite() or
     * updateWithTrueOverwrite().
     */
    void VegCostmapLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;

        auto node = node_.lock();
        if (!node)
            return;

        // Lock to prevent changes during update
        std::lock_guard<std::mutex> lock(mutex_);

        // Iterate through obstacle grid within the bounds
        for (int i = min_i; i < max_i; i++)
        {
            for (int j = min_j; j < max_j; j++)
            {
                const auto &obstacle = obstacle_grid_[i][j];
                if (obstacle.cost > 0)
                { // Only consider cells with obstacles
                    // Update the costmap layer with the new cost
                    updateCostValue_(costmap_, i, j, obstacle.cost);
                }
            }
        }
        // Apply to master grid with overwrite
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
    }

    /**
     * matchSize()
     * Method is called each time when map size was changed.
     */
    void VegCostmapLayer::matchSize()
    {
        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        // Call the parent method to resize the internal costmap
        CostmapLayer::matchSize();

        // Resize the obstacle_grid_ 2D vector to match the new dimensions
        obstacle_grid_.resize(map_width_);
        for (auto &row : obstacle_grid_)
        {
            row.resize(map_height_);
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Resized VegCostmapLayer to width: %u, height: %u",
            map_width_, map_height_);
    }

    /**
     * Method is called each time when footprint was changed.
     */
    void VegCostmapLayer::onFootprintChanged(void)
    {
        // This method is called when the footprint of the robot changes.
        // We can use it to update the obstacle radius if needed.
        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        // Update the obstacle radius based on the new footprint
        double footprint_radius = nav2_costmap_2d::getFootprintRadius(layered_costmap_->getFootprint());
        obstacle_radius_ = std::max(obstacle_radius_, footprint_radius);
    }

    /**
     * reset()
     * It may have any code to be executed during costmap reset.
     */
    void VegCostmapLayer::reset(void)
    {
        // Reset the obstacle grid
        for (auto &row : obstacle_grid_)
        {
            std::fill(row.begin(), row.end(), ObstaclePoint()); // Reset to default values
        }
        // Reset any state flags
        costmap_updated_ = false;
        // Reset the internal costmap (parent class)
        resetMaps();
        // Clear the obstacle database
        obstacle_database_.clear();
        // Re-initialize with default obstacle database if needed
        obstacle_database_ = veg_costmap::defaults::SAVED_OBSTACLE_DATABASE;
    };

    /**
     * @brief Get the cost of a saved obstacle from the database
     * Samples with a normal distribution to account for uncertainty, and bounds the cost to lethal cost
     * @param obstacle_name Name of the obstacle
     * @return Cost of the obstacle
     */
    double VegCostmapLayer::getSavedObstacleCost(std::string obstacle_name)
    {
        // Check if in the database
        if (obstacle_database_.find(obstacle_name) == obstacle_database_.end())
        {
            // Use unknown values if obstacle not found
            obstacle_database_[obstacle_name] = {{}, veg_costmap::defaults::UNKNOWN_COST, veg_costmap::defaults::UNKNOWN_COVARIANCE};
        }

        // Get the obstacle data from the database
        ObstacleData obstacle_data = obstacle_database_[obstacle_name];
        std::normal_distribution<double> normal_dist(obstacle_data.cost, obstacle_data.cost_stddev);
        // Sample from normal distribution
        double cost = normal_dist(gen);
        // Bound cost to lethal cost
        return std::min(cost, static_cast<double>(lethal_cost_));
    }

    void VegCostmapLayer::publishCostmapCallback()
    {
        auto node = node_.lock();
        if (!node || !enabled_)
            return;

        std::lock_guard<std::mutex> lock(mutex_);

        // Create an OccupancyGrid message
        auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

        // Set header
        grid_msg->header.stamp = node->now();
        grid_msg->header.frame_id = global_frame_;

        // Set map metadata
        grid_msg->info.width = costmap_->getSizeInCellsX();
        grid_msg->info.height = costmap_->getSizeInCellsY();
        grid_msg->info.resolution = costmap_->getResolution();
        grid_msg->info.origin.position.x = costmap_->getOriginX();
        grid_msg->info.origin.position.y = costmap_->getOriginY();
        grid_msg->info.origin.position.z = 0.0;
        grid_msg->info.origin.orientation.w = 1.0;

        // Resize data array
        grid_msg->data.resize(grid_msg->info.width * grid_msg->info.height);

        // Convert from costmap (0-255) to occupancy grid (-1 to 100)
        unsigned char *costmap_data = costmap_->getCharMap();
        for (unsigned int i = 0; i < grid_msg->data.size(); ++i)
        {
            unsigned char cost = costmap_data[i];

            if (cost == nav2_costmap_2d::NO_INFORMATION)
            {
                grid_msg->data[i] = -1; // Unknown
            }
            else if (cost == nav2_costmap_2d::FREE_SPACE)
            {
                grid_msg->data[i] = 0; // Free space
            }
            else if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
            {
                grid_msg->data[i] = 100; // Occupied
            }
            else
            {
                // Convert from 1-252 to 1-99 for better visualization
                grid_msg->data[i] = static_cast<int8_t>(1 + (cost * 98) / 252);
            }
        }

        // Publish the message
        costmap_pub_->publish(std::move(grid_msg));
    }

    /**
     * getWorldTransforms()
     * @param msg TF message containing the transforms of obstacles the world frame
     * Listen to transform messages to get the positions of vegetation obstacles in the world frame.
     * @return true if transforms were successfully retrieved and processed, false otherwise
     */
    bool VegCostmapLayer::getWorldTransforms(void)
    {
        auto node = node_.lock();
        if (!node)
        {
            RCLCPP_ERROR(rclcpp::get_logger("VegCostmapLayer"), "Cannot get transforms: node is not available");
            return false;
        }

        // Create client for the world TF service
        auto client = node->create_client<planner_msgs::srv::GetTransforms>("world_tf_service");

        // Wait for service to be available with timeout
        RCLCPP_INFO(node->get_logger(), "Waiting for world_tf_service to be available...");
        if (!client->wait_for_service(std::chrono::seconds(30)))
        {
            RCLCPP_ERROR(node->get_logger(), "Timed out waiting for world_tf_service");
            return false;
        }

        // Create request
        auto request = std::make_shared<planner_msgs::srv::GetTransforms::Request>();
        request->world = "outdoors";

        // Send request and wait for response
        RCLCPP_INFO(node->get_logger(), "Requesting transforms for world: outdoors");
        auto future = client->async_send_request(request);

        // Wait for response with timeout
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to receive response from world_tf_service");
            return false;
        }

        auto response = future.get();
        if (!response->success)
        {
            RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", response->message.c_str());
            return false;
        }

        auto transforms = response->transforms.transforms;

        std::lock_guard<std::mutex> lock(mutex_);

        for (const auto &transform : transforms)
        {
            const std::string &child_frame = transform.child_frame_id;

            // Skip if not vegetation - check if any vegetation name is in the child_frame
            bool is_vegetation = false;
            for (const auto &veg_name : veg_names_)
            {
                if (child_frame.find(veg_name) != std::string::npos)
                {
                    is_vegetation = true;
                    break;
                }
            }
            if (!is_vegetation)
                continue;

            // Store obstacle position from TF
            ObstaclePoint obstacle;
            obstacle.x = transform.transform.translation.x;
            obstacle.y = transform.transform.translation.y;
            obstacle.name = transform.child_frame_id;

            // Default to lethal cost for now
            obstacle.cost = VegCostmapLayer::getSavedObstacleCost(obstacle.name);

            // Add to the obstacles database
            obstacle_database_[obstacle.name].others.insert(obstacle);

            RCLCPP_DEBUG(
                node->get_logger(),
                "Added vegetation obstacle %s at (%.2f, %.2f) with cost %d",
                obstacle.name.c_str(), obstacle.x, obstacle.y, static_cast<int>(obstacle.cost));
        }

        costmap_updated_ = true;

        // Publish notification that costmap was updated
        auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
        replan_pub_->publish(std::move(msg_empty));

        return true;
    }

    /**
     * Updates a Gaussian distribution using Bayesian inference. Aka. Bayesian updating
     * This function combines a prior Gaussian distribution with a new observation
     * (also represented as a Gaussian) to produce an updated posterior distribution.
     *
     * @param prior_mean Mean of the prior distribution
     * @param prior_stddev Standard deviation of the prior distribution
     * @param obs_mean Mean of the new observation (or likelihood)
     * @param obs_stddev Standard deviation of the new observation
     * @param posterior_mean Pointer to store the updated mean
     * @param posterior_stddev Pointer to store the updated standard deviation
     */
    void bayesian_update_gaussian(
        double prior_mean, double prior_stddev,
        double obs_mean, double obs_stddev,
        unsigned char *posterior_mean, unsigned char *posterior_stddev)
    {
        // Calculate precisions (inverse of variance)
        double prior_precision = 1.0 / (prior_stddev * prior_stddev);
        double obs_precision = 1.0 / (obs_stddev * obs_stddev);

        // Calculate posterior precision (sum of precisions)
        double posterior_precision = prior_precision + obs_precision;

        // Calculate posterior mean (weighted by precisions)
        *posterior_mean = (prior_mean * prior_precision +
                           obs_mean * obs_precision) /
                          posterior_precision;

        // Calculate posterior standard deviation
        *posterior_stddev = sqrt(1.0 / posterior_precision);
    }
    double euclidean_distance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

} // namespace veg_costmap

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(veg_costmap::VegCostmapLayer, nav2_costmap_2d::Layer)