#include "planner/veg_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace veg_costmap
{

    VegCostmapLayer::VegCostmapLayer()
        : obstacle_range_(5.0),
          obstacle_radius_(0.5),
          clearing_enabled_(true),
          lethal_cost_(100),
          medium_cost_(70),
          low_cost_(50),
          costmap_updated_(false),
          use_gradient_costs_(true),
          gradient_factor_(0.8)
    {
    }

    void VegCostmapLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error("Failed to lock node");
        }

        // Declare parameters - using Layer's inherited methods
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("tf_topic", rclcpp::ParameterValue("/outdoors_tf"));
        declareParameter("veg_update_topic", rclcpp::ParameterValue("/veg_updates"));
        declareParameter("tf_prefix_filter", rclcpp::ParameterValue(""));
        declareParameter("obstacle_range", rclcpp::ParameterValue(5.0));
        declareParameter("obstacle_radius", rclcpp::ParameterValue(0.5));
        declareParameter("clearing_enabled", rclcpp::ParameterValue(true));
        declareParameter("lethal_cost", rclcpp::ParameterValue(100));
        declareParameter("medium_cost", rclcpp::ParameterValue(70));
        declareParameter("low_cost", rclcpp::ParameterValue(50));
        declareParameter("use_gradient_costs", rclcpp::ParameterValue(true));
        declareParameter("gradient_factor", rclcpp::ParameterValue(0.8));

        // Get parameters
        node->get_parameter(name_ + ".enabled", enabled_);
        node->get_parameter(name_ + ".tf_topic", tf_topic_);
        node->get_parameter(name_ + ".veg_update_topic", veg_update_topic_);
        node->get_parameter(name_ + ".tf_prefix_filter", tf_prefix_filter_);
        node->get_parameter(name_ + ".obstacle_range", obstacle_range_);
        node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
        node->get_parameter(name_ + ".clearing_enabled", clearing_enabled_);
        node->get_parameter(name_ + ".lethal_cost", lethal_cost_);
        node->get_parameter(name_ + ".medium_cost", medium_cost_);
        node->get_parameter(name_ + ".low_cost", low_cost_);
        node->get_parameter(name_ + ".use_gradient_costs", use_gradient_costs_);
        node->get_parameter(name_ + ".gradient_factor", gradient_factor_);

        global_frame_ = layered_costmap_->getGlobalFrameID();

        RCLCPP_INFO(
            node->get_logger(),
            "VegCostmapLayer initialized with tf_topic: %s, update_topic: %s, obstacle_range: %.2f, obstacle_radius: %.2f",
            tf_topic_.c_str(), veg_update_topic_.c_str(), obstacle_range_, obstacle_radius_);

        // Subscribe to TF messages for initial obstacle positions
        tf_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic_, 10,
            std::bind(&VegCostmapLayer::tfCallback, this, std::placeholders::_1));

        // Subscribe to vegetation costmap updates
        costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            veg_update_topic_, 10,
            std::bind(&VegCostmapLayer::costmapCallback, this, std::placeholders::_1));

        // Publisher to trigger replanning when costmap gets updated
        replan_pub_ = node->create_publisher<std_msgs::msg::Empty>(
            "/veg_costmap_updated", 1);

        current_ = true;
    }

    void VegCostmapLayer::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        obstacle_points_.clear();

        for (const auto &transform : msg->transforms)
        {
            // Skip if we have a prefix filter and this transform doesn't match
            if (!tf_prefix_filter_.empty() &&
                transform.child_frame_id.find(tf_prefix_filter_) != 0)
            {
                continue;
            }

            // Store obstacle position from TF
            ObstaclePoint obstacle;
            obstacle.x = transform.transform.translation.x;
            obstacle.y = transform.transform.translation.y;
            obstacle.name = transform.child_frame_id;

            // Default to lethal cost for now
            obstacle.cost = lethal_cost_;

            // Apply cost based on object height if available (lower height = less cost)
            // This assumes taller vegetation is harder to traverse
            double height = transform.transform.translation.z;
            if (height > 0)
            {
                if (height < 0.3)
                {
                    obstacle.cost = low_cost_;
                }
                else if (height < 1.0)
                {
                    obstacle.cost = medium_cost_;
                }
                // else keep lethal_cost_
            }

            obstacle_points_.push_back(obstacle);

            RCLCPP_DEBUG(
                node->get_logger(),
                "Added obstacle %s at (%.2f, %.2f) with cost %d",
                obstacle.name.c_str(), obstacle.x, obstacle.y, static_cast<int>(obstacle.cost));
        }

        costmap_updated_ = true;

        // Publish notification that costmap was updated
        auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
        replan_pub_->publish(std::move(msg_empty));
    }

    void VegCostmapLayer::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // Check if frame IDs match
        if (msg->header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Received vegetation update in frame %s, but costmap is in frame %s",
                msg->header.frame_id.c_str(), global_frame_.c_str());
            return;
        }

        // Get costmap dimensions and resolution
        double resolution = msg->info.resolution;
        unsigned int width = msg->info.width;
        unsigned int height = msg->info.height;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        // Clear existing obstacles
        obstacle_points_.clear();

        // Process each cell in the received grid
        for (unsigned int j = 0; j < height; ++j)
        {
            for (unsigned int i = 0; i < width; ++i)
            {
                int cost_value = msg->data[j * width + i];

                // Skip free space
                if (cost_value <= 0)
                {
                    continue;
                }

                // Calculate world coordinates for this cell
                double world_x = origin_x + (i + 0.5) * resolution;
                double world_y = origin_y + (j + 0.5) * resolution;

                // Create obstacle point
                ObstaclePoint obstacle;
                obstacle.x = world_x;
                obstacle.y = world_y;
                obstacle.name = "veg_" + std::to_string(i) + "_" + std::to_string(j);

                // Map from OccupancyGrid values (0-100) to costmap costs
                if (cost_value > 90)
                {
                    obstacle.cost = lethal_cost_;
                }
                else if (cost_value > 70)
                {
                    obstacle.cost = medium_cost_;
                }
                else if (cost_value > 0)
                {
                    obstacle.cost = low_cost_;
                }

                obstacle_points_.push_back(obstacle);
            }
        }

        costmap_updated_ = true;

        // Publish notification that costmap was updated
        auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
        replan_pub_->publish(std::move(msg_empty));

        RCLCPP_INFO(
            node->get_logger(),
            "Updated vegetation costmap with %zu obstacles",
            obstacle_points_.size());
    }

    void VegCostmapLayer::updateVegetationCost(unsigned int mx, unsigned int my, unsigned char cost)
    {
        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // Get world coordinates
        double wx, wy;
        layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);

        // Find if this point already exists
        bool found = false;
        for (auto &point : obstacle_points_)
        {
            unsigned int point_mx, point_my;
            if (layered_costmap_->getCostmap()->worldToMap(point.x, point.y, point_mx, point_my))
            {
                if (point_mx == mx && point_my == my)
                {
                    point.cost = cost;
                    found = true;
                    break;
                }
            }
        }

        // Add new point if not found
        if (!found)
        {
            ObstaclePoint obstacle;
            obstacle.x = wx;
            obstacle.y = wy;
            obstacle.name = "veg_" + std::to_string(mx) + "_" + std::to_string(my);
            obstacle.cost = cost;
            obstacle_points_.push_back(obstacle);
        }

        costmap_updated_ = true;

        // Publish notification that costmap was updated
        auto msg_empty = std::make_unique<std_msgs::msg::Empty>();
        replan_pub_->publish(std::move(msg_empty));
    }

    void VegCostmapLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y)
    {
        (void)robot_yaw; // Unused parameter
        if (!enabled_ || obstacle_points_.empty())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // Reset bounds to current robot position initially
        double new_min_x = robot_x;
        double new_min_y = robot_y;
        double new_max_x = robot_x;
        double new_max_y = robot_y;

        // Expand bounds to include all obstacles within range
        for (const auto &obstacle : obstacle_points_)
        {
            double distance = veg_costmap::distance(robot_x, robot_y, obstacle.x, obstacle.y);

            if (distance <= obstacle_range_)
            {
                new_min_x = std::min(new_min_x, obstacle.x - obstacle_radius_);
                new_min_y = std::min(new_min_y, obstacle.y - obstacle_radius_);
                new_max_x = std::max(new_max_x, obstacle.x + obstacle_radius_);
                new_max_y = std::max(new_max_y, obstacle.y + obstacle_radius_);
            }
        }

        // Update the bounds only if costmap has changed
        if (costmap_updated_)
        {
            *min_x = std::min(*min_x, new_min_x);
            *min_y = std::min(*min_y, new_min_y);
            *max_x = std::max(*max_x, new_max_x);
            *max_y = std::max(*max_y, new_max_y);
            costmap_updated_ = false;
        }
    }

    void VegCostmapLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            return;
        }

        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        // Lock to prevent changes to obstacle_points_ during update
        std::lock_guard<std::mutex> lock(mutex_);

        // Clear the specified region in the costmap if clearing is enabled
        if (clearing_enabled_)
        {
            for (int j = min_j; j < max_j; j++)
            {
                for (int i = min_i; i < max_i; i++)
                {
                    unsigned char *cell = master_grid.getCharMap() + master_grid.getIndex(i, j);
                    if (*cell == nav2_costmap_2d::LETHAL_OBSTACLE)
                    {
                        *cell = nav2_costmap_2d::FREE_SPACE;
                    }
                }
            }
        }

        // For each obstacle, mark cells with appropriate cost
        for (const auto &obstacle : obstacle_points_)
        {
            // Get map coordinates for obstacle
            unsigned int mx, my;
            if (!master_grid.worldToMap(obstacle.x, obstacle.y, mx, my))
            {
                RCLCPP_DEBUG(
                    node->get_logger(),
                    "Obstacle at (%.2f, %.2f) is off the map",
                    obstacle.x, obstacle.y);
                continue;
            }

            // Calculate the bounds of the obstacle
            unsigned int radius_cells = std::max(1u, static_cast<unsigned int>(obstacle_radius_ / master_grid.getResolution()));

            // Apply gradient cost if enabled, otherwise use fixed cost
            if (use_gradient_costs_)
            {
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
                        if (cur_mx < 0 || cur_mx >= master_grid.getSizeInCellsX() ||
                            cur_my < 0 || cur_my >= master_grid.getSizeInCellsY())
                        {
                            continue;
                        }

                        // Calculate cost based on distance from center (higher at center, lower at edges)
                        double distance_factor = 1.0 - (std::sqrt(distance_sq) / radius_cells);
                        unsigned char new_cost = static_cast<unsigned char>(
                            obstacle.cost * (distance_factor * gradient_factor_ + (1.0 - gradient_factor_)));

                        unsigned char old_cost = master_grid.getCost(cur_mx, cur_my);
                        if (old_cost == nav2_costmap_2d::NO_INFORMATION)
                        {
                            continue;
                        }

                        // Apply the maximum cost between existing and new
                        master_grid.setCost(cur_mx, cur_my, std::max(old_cost, new_cost));
                    }
                }
            }
            else
            {
                // Simple fixed cost application
                for (int i = -static_cast<int>(radius_cells); i <= static_cast<int>(radius_cells); i++)
                {
                    for (int j = -static_cast<int>(radius_cells); j <= static_cast<int>(radius_cells); j++)
                    {
                        // Skip cells outside the circle
                        if (static_cast<unsigned int>(i * i + j * j) > radius_cells * radius_cells)
                        {
                            continue;
                        }

                        unsigned int cur_mx = mx + i;
                        unsigned int cur_my = my + j;

                        // Check if cell is within map bounds
                        if (cur_mx < 0 || cur_mx >= master_grid.getSizeInCellsX() ||
                            cur_my < 0 || cur_my >= master_grid.getSizeInCellsY())
                        {
                            continue;
                        }

                        // Mark the cell with the obstacle's cost
                        unsigned char old_cost = master_grid.getCost(cur_mx, cur_my);
                        if (old_cost == nav2_costmap_2d::NO_INFORMATION)
                        {
                            continue;
                        }

                        master_grid.setCost(cur_mx, cur_my, std::max(old_cost, obstacle.cost));
                    }
                }
            }
        }
    }

    double distance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

} // namespace veg_costmap

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(veg_costmap::VegCostmapLayer, nav2_costmap_2d::Layer)