#include "planner/veg_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "planner_msgs/srv/update_cost.hpp"
#include "planner_msgs/srv/get_transforms.hpp"
#include <string>

/**
 * @file veg_costmap_layer.cpp
 * @brief Implementation of a costmap layer for dynamic vegetation environments.
 * Custom nav2 costmap: https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html
 * Sample code: https://github.com/ros-navigation/navigation2_tutorials/blob/humble/nav2_gradient_costmap_plugin/src/gradient_layer.cpp
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
    // Initialize the global logger and mutex
    rclcpp::Logger logger_{rclcpp::get_logger("VegCostmapLayer")};

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

        RCLCPP_INFO(logger_, "Initializing VegCostmapLayer...");

        // Restart with a reset
        reset();

        // Declare parameters - using Layer's inherited methods
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("world_tf_service", rclcpp::ParameterValue("/world/get_tf"));
        declareParameter("update_topic", rclcpp::ParameterValue("/veg_costmap/update"));
        declareParameter("updated_topic", rclcpp::ParameterValue("/veg_costmap/updated"));
        declareParameter("costmap_topic", rclcpp::ParameterValue("/veg_costmap"));
        declareParameter("obstacle_range", rclcpp::ParameterValue(5.0));
        declareParameter("obstacle_radius", rclcpp::ParameterValue(20.0));
        declareParameter("lethal_cost", rclcpp::ParameterValue(254));
        declareParameter("use_gradient_costs", rclcpp::ParameterValue(true));
        declareParameter("gradient_factor", rclcpp::ParameterValue(0.8));

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

        // Initialize the map settings before accessing the costmap
        if (!layered_costmap_ || !layered_costmap_->getCostmap())
        {
            RCLCPP_ERROR(logger_, "Layered costmap not initialized properly");
            throw std::runtime_error("Failed to initialize layered costmap");
        }

        // Set map attributes
        map_width_ = layered_costmap_->getCostmap()->getSizeInCellsX();    // in cells
        map_height_ = layered_costmap_->getCostmap()->getSizeInCellsY();   // in cells
        map_resolution_ = layered_costmap_->getCostmap()->getResolution(); // in meters
        map_origin_x_ = layered_costmap_->getCostmap()->getOriginX();      // in meters
        map_origin_y_ = layered_costmap_->getCostmap()->getOriginY();      // in meters
        RCLCPP_INFO(
            logger_,
            "Costmap size: %u x %u cells, resolution: %.2f m, origin: (%.2f, %.2f)",
            map_width_, map_height_, map_resolution_, map_origin_x_, map_origin_y_);

        global_frame_ = layered_costmap_->getGlobalFrameID();

        // Check for valid dimensions to prevent segfaults
        if (map_width_ == 0 || map_height_ == 0)
        {
            RCLCPP_ERROR(logger_, "Invalid costmap dimensions: %d x %d",
                         map_width_, map_height_);
            throw std::runtime_error("Invalid costmap dimensions");
        }

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

        // Init obstacle grid 2D with safe allocation
        try
        {
            obstacle_grid_.resize(static_cast<size_t>(map_width_));
            for (auto &row : obstacle_grid_)
            {
                row.resize(static_cast<size_t>(map_height_), ObstaclePoint());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to allocate obstacle grid: %s", e.what());
            throw std::runtime_error("Failed to allocate obstacle grid");
        }

        // Initialize obstacle_database_ with proper lock
        {
            // std::lock_guard<std::mutex> lock(mutex_);
            // Init the obstacle_database_ unordered_map with prior knowledge
            obstacle_database_ = veg_costmap::defaults::SAVED_OBSTACLE_DATABASE;
        }

        try
        {
            // Get world vegetated obstacles
            bool success = VegCostmapLayer::getWorldTransforms();
            if (!success)
            {
                RCLCPP_WARN(logger_, "Failed to get world transforms, continuing with empty map");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Exception during world transform fetch: %s", e.what());
            RCLCPP_WARN(logger_, "Continuing with initialization despite transform error");
            // Don't rethrow - continue with initialization even if this fails
        }

        RCLCPP_INFO(logger_, "VegCostmapLayer initialized");
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

        RCLCPP_INFO(logger_, "Entering updateCostCallback...");

        // Error checking
        if (!node)
        {
            RCLCPP_ERROR(logger_, "Cannot update cost: node is not available");
            response->success = false;
            response->message = "Node is not available";
            return;
        }
        if (!request)
        {
            RCLCPP_ERROR(logger_, "Received null request");
            response->success = false;
            response->message = "Null request received";
            return;
        }

        // Get the index and cost from the request
        float world_x = request->x;
        float world_y = request->y;
        unsigned char new_cost = request->cost;                                  // unsigned char = uint8_t (0-255)
        new_cost = std::min(new_cost, static_cast<unsigned char>(lethal_cost_)); // Bound cost to lethal cost
        std::string obstacle_type = request->obstacle_type;

        nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();

        // Check if layered_costmap_ and its costmap are valid
        if (!layered_costmap_ || !costmap)
        {
            RCLCPP_ERROR(logger_, "Layered costmap not initialized");
            response->success = false;
            response->message = "Layered costmap not initialized";
            return;
        }

        // Convert to map coordinates
        unsigned int map_x, map_y;
        if (!costmap->worldToMap(world_x, world_y, map_x, map_y))
        {
            response->success = false;
            response->message = "Coordinates out of map bounds";
            return;
        }

        // Check coordinates are within grid bounds (double check)
        if (map_x >= map_width_ || map_y >= map_height_)
        {
            RCLCPP_ERROR(logger_, "Converted coordinates (%u, %u) out of bounds (%u, %u)",
                         map_x, map_y, map_width_, map_height_);
            response->success = false;
            response->message = "Converted coordinates out of bounds";
            return;
        }

        // // Thread safety for accessing obstacle grid
        // std::lock_guard<std::mutex> lock(mutex_);

        ObstaclePoint &obstacle = obstacle_grid_[map_x][map_y];

        if (obstacle.x != map_x || obstacle.y != map_y)
        {
            // If the obstacle is not already in the grid, add it
            obstacle.x = world_x;
            obstacle.y = world_y;
            obstacle.mx = map_x;
            obstacle.my = map_y;
            obstacle.name = obstacle_type;

            // Add to the obstacle database
            obstacle_database_[obstacle_type].others.insert(obstacle);
        }

        // Update the obstacle grid with the code
        obstacle.cost = new_cost;
        obstacle.cost_stddev = 0; // Mark as already sampled!

        // If obstacle has name, update the obstacle_database_ with the new cost
        if (!obstacle_type.empty())
        {
            RCLCPP_INFO(logger_, "Updating for other obstacles of type %s", obstacle_type.c_str());

            ObstacleData &obstacle_data = obstacle_database_[obstacle_type];
            if (obstacle_data.cost == 0)
            {
                // No prior data, set initial cost and stddev
                obstacle_data.cost = new_cost;
                obstacle_data.cost_stddev = obstacle_database_[obstacle_type].cost_stddev;
            }
            else
            {
                // Combine the new normal dist with the prior normal dist
                bayesianUpdateGaussian(
                    // Old prior knowledge
                    obstacle_data.cost, obstacle_data.cost_stddev,
                    // New sample, assume twice as accurate as prior (ie. divide by 2)
                    new_cost, obstacle_data.cost_stddev / 2.0,
                    // Output updated mean and stddev
                    &obstacle_data.cost, &obstacle_data.cost_stddev);
            }

            // UPDATE similar obstacles of the same type
            // Iterate through all the other similar obstacles and update their cost
            for (auto &obstacle : obstacle_data.others)
            {
                // If stddev is not 0 (ie. not already sampled), update cost from a normal distribution
                bool is_sampled = obstacle_grid_[obstacle.mx][obstacle.my].cost_stddev == 0;
                if (!is_sampled)
                {
                    std::normal_distribution<double> normal_dist(obstacle_data.cost, obstacle_data.cost_stddev);
                    // Sample from normal distribution
                    double sampled_cost = normal_dist(gen);
                    // Ensure cost is within valid range (0-255)

                    // IMPORTANT: Use mx and my for grid coordinates, not x and y
                    if (obstacle.mx < map_width_ && obstacle.my < map_height_)
                    {
                        obstacle_grid_[obstacle.mx][obstacle.my].cost =
                            static_cast<unsigned char>(clampCost(sampled_cost));
                    }
                }
            }
        }

        costmap_updated_ = true;

        // Publish notification that costmap was updated
        auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
        replan_pub_->publish(std::move(msg_empty));

        // Set response
        response->success = true;
        response->message = "Cost updated successfully";

        RCLCPP_INFO(logger_, "Exiting updateCostCallback");
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

        RCLCPP_DEBUG(logger_, "Entering updateBounds...");

        // Unused parameters
        (void)robot_x;
        (void)robot_y;
        (void)robot_yaw;

        if (!enabled_ || !min_x || !min_y || !max_x || !max_y)
            return;

        if (obstacle_grid_.empty() || obstacle_grid_.size() != map_width_)
        {
            RCLCPP_WARN(logger_, "Obstacle grid not properly initialized yet, skipping bounds update");
            return;
        }

        // // Add mutex protection for obstacle grid access
        // std::lock_guard<std::mutex> lock(mutex_);

        // Option 2: Loop through obstacle_grid_ to find bounds of changed areas
        // This is more efficient if changes are localized
        bool found_update = false;
        double update_min_x = std::numeric_limits<double>::max();
        double update_min_y = std::numeric_limits<double>::max();
        double update_max_x = std::numeric_limits<double>::lowest();
        double update_max_y = std::numeric_limits<double>::lowest();

        // Check if costmap and layered_costmap_ pointers are valid
        if (!layered_costmap_ || !layered_costmap_->getCostmap())
        {
            RCLCPP_ERROR(logger_, "Layered costmap not initialized in updateBounds");
            return;
        }

        // Make sure obstacle_grid_ dimensions match the expected size
        if (obstacle_grid_.size() != map_width_)
        {
            RCLCPP_ERROR(logger_, "Obstacle grid width mismatch: %zu vs %u",
                         obstacle_grid_.size(), map_width_);
            return;
        }

        for (size_t i = 0; i < obstacle_grid_.size(); i++)
        {
            if (obstacle_grid_[i].size() != map_height_)
            {
                RCLCPP_ERROR(logger_, "Obstacle grid height mismatch at row %zu: %zu vs %u",
                             i, obstacle_grid_[i].size(), map_height_);
                return;
            }

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
        else
        {
            // No updates found, set bounds to the entire costmap
            *min_x = map_origin_x_;
            *min_y = map_origin_y_;
            *max_x = map_origin_x_ + (map_width_ * map_resolution_);
            *max_y = map_origin_y_ + (map_height_ * map_resolution_);
        }

        RCLCPP_DEBUG(logger_, "Exiting updateBounds");
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

        RCLCPP_DEBUG(logger_, "Entering updateCosts...");

        // Validate input bounds
        unsigned int size_x = master_grid.getSizeInCellsX();
        unsigned int size_y = master_grid.getSizeInCellsY();

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        if (min_i > max_i || min_j > max_j)
        {
            RCLCPP_ERROR(logger_, "Invalid update bounds: min(%d, %d), max(%d, %d)",
                         min_i, min_j, max_i, max_j);
            return;
        }

        // Get direct pointer to the master grid
        unsigned char *master_array = master_grid.getCharMap();
        if (!master_array)
        {
            RCLCPP_ERROR(logger_, "Invalid master array pointer");
            return;
        }

        // // Thread safety for accessing obstacle database
        // std::lock_guard<std::mutex> lock(mutex_);

        // Then update the master grid directly with all obstacles
        int count = 0;
        for (const auto &obstacle_pair : obstacle_database_)
        {
            for (const auto &point : obstacle_pair.second.others)
            {
                // Validate point coordinates
                if (point.mx < size_x && point.my < size_y)
                {
                    // Bound the cost to [0, 255]
                    unsigned char cost = static_cast<unsigned char>(clampCost(obstacle_pair.second.cost));

                    // Set cost directly in master grid
                    // Set constant cost for the obstacle around its radius by obstacle_radius_
                    for (int dx = -obstacle_radius_; dx <= obstacle_radius_; ++dx)
                    {
                        for (int dy = -obstacle_radius_; dy <= obstacle_radius_; ++dy)
                        {
                            // Check if the cell is within the circular radius
                            if (dx * dx + dy * dy <= obstacle_radius_ * obstacle_radius_)
                            {
                                int new_index = master_grid.getIndex(point.mx + dx, point.my + dy);
                                if (new_index >= 0 && new_index < static_cast<int>(size_x * size_y))
                                {
                                    master_array[new_index] = cost;
                                }
                            }
                        }
                    }

                    count++;
                }
            }
        }

        // Log how many obstacles we're updating
        RCLCPP_DEBUG(logger_, "Updating %d obstacles in costmap", count);

        RCLCPP_DEBUG(logger_, "Exiting updateCosts");
    }

    /**
     * reset()
     * It may have any code to be executed during costmap reset.
     */
    void VegCostmapLayer::reset(void)
    {
        RCLCPP_INFO(logger_, "Entering reset...");

        // // Thread safety for accessing obstacle grid and database
        // std::lock_guard<std::mutex> lock(mutex_);

        // Reset the obstacle grid
        try
        {
            // Resize and initialize grid only if map dimensions are valid
            if (map_width_ > 0 && map_height_ > 0)
            {
                obstacle_grid_.resize(map_width_);
                for (auto &row : obstacle_grid_)
                {
                    row.resize(map_height_, ObstaclePoint()); // Reset to default values
                }
            }
            else
            {
                RCLCPP_WARN(logger_, "Skipping obstacle grid initialization due to invalid dimensions: %u x %u",
                            map_width_, map_height_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Exception during grid reset: %s", e.what());
        }

        // Reset any state flags
        costmap_updated_ = false;

        // Reset the internal costmap (parent class)
        resetMaps();

        // Clear the obstacle database
        obstacle_database_.clear();

        // Re-initialize with default obstacle database if needed
        obstacle_database_ = veg_costmap::defaults::SAVED_OBSTACLE_DATABASE;

        RCLCPP_INFO(logger_, "Exiting reset...");
    }

    void VegCostmapLayer::publishCostmapCallback()
    {
        auto node = node_.lock();
        if (!node || !enabled_)
            return;

        // Check if layered_costmap_ and its costmap are valid
        if (!layered_costmap_ || !layered_costmap_->getCostmap())
        {
            RCLCPP_ERROR(logger_, "Layered costmap not initialized in publishCostmapCallback");
            return;
        }

        // Create an OccupancyGrid message
        auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

        // Set header
        grid_msg->header.stamp = node->now();
        grid_msg->header.frame_id = global_frame_;

        // Set map metadata
        grid_msg->info.width = map_width_;
        grid_msg->info.height = map_height_;
        grid_msg->info.resolution = map_resolution_;
        grid_msg->info.origin.position.x = map_origin_x_;
        grid_msg->info.origin.position.y = map_origin_y_;
        grid_msg->info.origin.position.z = 0.0;
        grid_msg->info.origin.orientation.w = 1.0;

        // Resize data array
        grid_msg->data.resize(grid_msg->info.width * grid_msg->info.height);

        // Get costmap data ptr safely
        unsigned char *costmap_data = layered_costmap_->getCostmap()->getCharMap();
        if (!costmap_data)
        {
            RCLCPP_ERROR(logger_, "Null costmap data in publishCostmapCallback");
            return;
        }

        // Convert from costmap (0-255) to occupancy grid (-1 to 100)
        for (unsigned int i = 0; i < grid_msg->data.size(); ++i)
        {
            // Bounds check for safety
            if (i >= map_width_ * map_height_)
            {
                RCLCPP_ERROR(logger_, "Index out of bounds in costmap conversion: %u >= %u",
                             i, map_width_ * map_height_);
                break;
            }

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
     * Updates the obstacle_database and marks the costmap for update
     * @return true if transforms were successfully retrieved and processed, false otherwise
     */
    bool VegCostmapLayer::getWorldTransforms(void)
    {
        auto node = node_.lock();
        if (!node)
        {
            RCLCPP_ERROR(logger_, "Cannot get transforms: node is not available");
            return false;
        }

        // Check if the layered_costmap_ and its costmap are valid
        if (!layered_costmap_ || !layered_costmap_->getCostmap())
        {
            RCLCPP_ERROR(logger_, "Layered costmap not initialized in getWorldTransforms");
            return false;
        }

        // Create client for the world TF service
        auto client = node->create_client<planner_msgs::srv::GetTransforms>(world_tf_service_);

        // Wait for service to be available with timeout
        RCLCPP_INFO(logger_, "Waiting for world_tf_service to be available at topic: %s",
                    world_tf_service_.c_str());
        if (!client->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(logger_, "Timed out waiting for world_tf_service");
            return false;
        }

        // Create request
        auto request = std::make_shared<planner_msgs::srv::GetTransforms::Request>();
        request->world = "outdoors";

        // Send request and wait for response
        RCLCPP_INFO(logger_, "Requesting transforms for world: outdoors");
        auto future = client->async_send_request(request);

        // Wait for response with timeout
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(20)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(logger_, "Failed to receive response from world_tf_service");
            return false;
        }

        auto response = future.get();
        if (!response || !response->success)
        {
            RCLCPP_ERROR(logger_, "Service call failed: %s",
                         response ? response->message.c_str() : "null response");
            return false;
        }

        auto transforms = response->transforms.transforms;
        RCLCPP_INFO(logger_, "Received %zu transforms", transforms.size());

        int num_obstacles = 0;

        // // Thread safety for accessing obstacle database
        // std::lock_guard<std::mutex> lock(mutex_);

        // Process transforms in a single loop
        for (const auto &transform : transforms)
        {
            const std::string &child_frame = transform.child_frame_id;
            RCLCPP_INFO(logger_, "Processing transform for: %s", child_frame.c_str());

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
            {
                RCLCPP_INFO(logger_, "Skipping non-vegetation transform: %s", child_frame.c_str());
                continue;
            }

            RCLCPP_INFO(logger_, "Found vegetation transform: %s", child_frame.c_str());

            // Get the obstacle data from the database
            ObstacleData &obstacle_data = obstacle_database_[child_frame];

            // Use safe normal distribution with proper bounds checking
            double mean_cost = clampCost(obstacle_data.cost);
            double stddev = std::max(0.1, static_cast<double>(obstacle_data.cost_stddev)); // Ensure positive stddev

            std::normal_distribution<double> normal_dist(mean_cost, stddev);
            double obstacle_cost = normal_dist(gen);
            obstacle_cost = clampCost(obstacle_cost);

            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;

            RCLCPP_INFO(logger_, "Vegetation at (%.2f, %.2f) with cost %f", x, y, obstacle_cost);

            // Convert world coordinates to map coordinates
            unsigned int mx, my;
            if (!layered_costmap_->getCostmap()->worldToMap(x, y, mx, my))
            {
                RCLCPP_ERROR(logger_, "Failed to convert world coordinates to map coordinates: (%.2f, %.2f)",
                             x, y);
                continue;
            }

            // Verify converted coordinates are within bounds
            if (mx >= map_width_ || my >= map_height_)
            {
                RCLCPP_ERROR(logger_, "Converted coordinates outside map bounds: (%u, %u) > (%u, %u)",
                             mx, my, map_width_, map_height_);
                continue;
            }

            // Create and add the obstacle point to the database directly
            ObstaclePoint obstacle_point;
            obstacle_point.x = x;
            obstacle_point.y = y;
            obstacle_point.mx = mx;
            obstacle_point.my = my;
            obstacle_point.name = child_frame;
            obstacle_point.cost = static_cast<unsigned char>(obstacle_cost);
            obstacle_point.cost_stddev = obstacle_database_[child_frame].cost_stddev;

            // Add to obstacles database - ensure others set is initialized first
            if (!obstacle_database_[child_frame].others.insert(obstacle_point).second)
            {
                RCLCPP_INFO(logger_, "Obstacle %s at (%.2f, %.2f) already exists in database",
                            child_frame.c_str(), x, y);
            }
            else
            {
                RCLCPP_INFO(
                    logger_,
                    "Added vegetation obstacle %s at (%.2f, %.2f) with cost %d",
                    child_frame.c_str(), x, y, static_cast<int>(obstacle_point.cost));
                num_obstacles++;
            }

            // Add to obstacle grid
            obstacle_grid_[mx][my] = obstacle_point;
        }

        if (num_obstacles > 0)
        {
            costmap_updated_ = true;

            // Publish notification that costmap was updated
            auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
            replan_pub_->publish(std::move(msg_empty));
        }

        RCLCPP_INFO(
            logger_,
            "Saved %d vegetation obstacles from world to database",
            num_obstacles);

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
    void VegCostmapLayer::bayesianUpdateGaussian(
        double prior_mean, double prior_stddev,
        double obs_mean, double obs_stddev,
        unsigned char *posterior_mean, unsigned char *posterior_stddev)
    {
        if (!posterior_mean || !posterior_stddev)
        {
            RCLCPP_ERROR(logger_, "Null pointers provided to bayesianUpdateGaussian");
            return;
        }

        // Ensure positive stddevs to prevent division by zero
        prior_stddev = std::max(0.1, prior_stddev);
        obs_stddev = std::max(0.1, obs_stddev);

        // Calculate precisions (inverse of variance)
        double prior_precision = 1.0 / (prior_stddev * prior_stddev);
        double obs_precision = 1.0 / (obs_stddev * obs_stddev);

        // Calculate posterior precision (sum of precisions)
        double posterior_precision = prior_precision + obs_precision;

        // Calculate posterior mean (weighted by precisions)
        double post_mean = (prior_mean * prior_precision +
                            obs_mean * obs_precision) /
                           posterior_precision;

        // Ensure the result is within the valid range for unsigned char (0-255)
        post_mean = clampCost(post_mean);

        // Calculate posterior standard deviation
        double post_stddev = sqrt(1.0 / posterior_precision);

        // Store results with proper bounds checking for unsigned char
        *posterior_mean = static_cast<unsigned char>(post_mean);
        *posterior_stddev = static_cast<unsigned char>(std::min(static_cast<double>(lethal_cost_), post_stddev));
    }

    /**
     * @brief Clamps the cost to be within the range [0, lethal_cost_]
     * @param cost The cost to clamp
     * @return The clamped cost
     */
    double VegCostmapLayer::clampCost(double cost)
    {
        return std::max(0.0, std::min(static_cast<double>(veg_costmap::VegCostmapLayer::lethal_cost_), cost));
    }

} // namespace veg_costmap

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(veg_costmap::VegCostmapLayer, nav2_costmap_2d::Layer)