#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include "nav2_util/node_utils.hpp"
#include "planner/vegmap_planner.hpp"

/**
 * @file vegmap_planner.cpp
 * @brief Adaption of the D* Lite global planner for dynamic vegetation environments.
 * Custom Nav2 Planner: https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html
 * Custom plugin sample code: https://github.com/ros-navigation/navigation2_tutorials/blob/humble/nav2_straightline_planner/src/straight_line_planner.cpp
 */
namespace vegmap_planner
{
    /**
     * configure()
     * Method is called at when planner server enters on_configure state.
     * Ideally this methods should perform declarations of ROS parameters and initialization of planner’s member variables.
     * @param parent shared pointer to parent node
     * @param name planner name
     * @param tf tf buffer pointer
     * @param costmap_ros shared pointer to costmap
     */
    void VegmapPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        RCLCPP_INFO(
            node_->get_logger(), "Configuring plugin %s of type VegmapPlanner...",
            name.c_str());

        // D* Lite specific parameters
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
        node_->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".lethal_cost", rclcpp::ParameterValue(253));
        node_->get_parameter(name_ + ".lethal_cost", lethal_cost_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".obstacle_range", rclcpp::ParameterValue(254));
        node_->get_parameter(name_ + ".obstacle_range", obstacle_range_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".costmap_updated_topic", rclcpp::ParameterValue("/veg_costmap/updated"));
        node_->get_parameter(name_ + ".costmap_updated_topic", costmap_updated_topic_);

        // Subscribe to costmap update notifications
        costmap_update_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
            costmap_updated_topic_, 1,
            std::bind(&VegmapPlanner::costmapUpdateCallback, this, std::placeholders::_1));

        // Initialize D* lite data structures
        reset();

        RCLCPP_INFO(
            node_->get_logger(), "Configured plugin %s of type VegmapPlanner", name_.c_str());
    }

    /**
     * activate()
     * Method is called when planner server enters on_activate state.
     * Ideally this method should implement operations which are necessary before planner goes to an active state.
     */
    void VegmapPlanner::activate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating plugin %s of type VegmapPlanner",
            name_.c_str());

        // Reset the costmap change flag
        costmap_changed_ = false;
    }

    /**
     * deactivate()
     * Method is called when planner server enters on_deactivate state.
     * Ideally this method should implement operations which are necessary before planner goes to an inactive state.
     */
    void VegmapPlanner::deactivate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating plugin %s of type VegmapPlanner",
            name_.c_str());

        // Clear active goal state
        has_active_goal_ = false;
    }

    /**
     * cleanup()
     * Method is called when planner server goes to on_cleanup state.
     */
    void VegmapPlanner::cleanup()
    {
        RCLCPP_INFO(
            node_->get_logger(), "CleaningUp plugin %s of type VegmapPlanner",
            name_.c_str());
        reset();
    }

    /**
     * reset()
     * Resets all the data structures used by D* Lite algorithm
     */
    void VegmapPlanner::reset()
    {
        g_values_.clear();
        rhs_values_.clear();
        open_list_ = std::priority_queue<StateKey>();
        km_ = 0.0;
        last_start_ = {-1, -1};
        last_goal_ = {-1, -1};
    }

    /**
     * calculateHeuristic()
     * Calculates heuristic distance between two cells using Euclidean distance
     */
    double VegmapPlanner::calculateHeuristic(const CellIndex &a, const CellIndex &b)
    {
        return heuristic_weight_ * std::hypot(a.x - b.x, a.y - b.y);
    }

    /**
     * calculateKey()
     * Calculates the priority key for a given cell
     */
    StateKey VegmapPlanner::calculateKey(const CellIndex &cell)
    {
        double g = getGValue(cell);
        double rhs = getRhsValue(cell);
        double min_val = std::min(g, rhs);

        StateKey key;
        key.k1 = min_val + calculateHeuristic(cell, last_start_) + km_;
        key.k2 = min_val;
        key.cell = cell;
        return key;
    }

    /**
     * getGValue()
     * Gets g-value for a cell, returns infinity if not set
     */
    double VegmapPlanner::getGValue(const CellIndex &cell)
    {
        auto it = g_values_.find(cell);
        if (it != g_values_.end())
        {
            return it->second;
        }
        return std::numeric_limits<double>::infinity();
    }

    /**
     * getRhsValue()
     * Gets rhs-value for a cell, returns infinity if not set
     * except for goal cell which returns 0
     */
    double VegmapPlanner::getRhsValue(const CellIndex &cell)
    {
        if (cell == last_goal_)
        {
            return 0.0;
        }

        auto it = rhs_values_.find(cell);
        if (it != rhs_values_.end())
        {
            return it->second;
        }
        return std::numeric_limits<double>::infinity();
    }

    /**
     * updateVertex()
     * Updates vertex information and open list if necessary
     */
    void VegmapPlanner::updateVertex(const CellIndex &cell)
    {
        if (!(cell == last_goal_))
        {
            // Calculate minimum rhs value based on successors
            double min_rhs = std::numeric_limits<double>::infinity();
            for (const auto &succ : getNeighbors(cell))
            {
                double cost = getCost(cell, succ);
                if (cost < std::numeric_limits<double>::infinity())
                {
                    min_rhs = std::min(min_rhs, getGValue(succ) + cost);
                }
            }
            rhs_values_[cell] = min_rhs;
        }

        // Update open list
        std::priority_queue<StateKey> new_queue;
        while (!open_list_.empty())
        {
            StateKey current = open_list_.top();
            open_list_.pop();

            if (!(current.cell == cell))
            {
                new_queue.push(current);
            }
        }
        open_list_ = new_queue;

        if (getGValue(cell) != getRhsValue(cell))
        {
            StateKey key = calculateKey(cell);
            key.cell = cell;
            open_list_.push(key);
        }
    }

    /**
     * computeShortestPath()
     * Main D* Lite algorithm to compute the shortest path
     */
    void VegmapPlanner::computeShortestPath()
    {
        while (!open_list_.empty() &&
               (open_list_.top().k1 < calculateKey(last_start_).k1 ||
                getRhsValue(last_start_) != getGValue(last_start_)))
        {

            StateKey current = open_list_.top();
            open_list_.pop();

            CellIndex cell = current.cell;
            StateKey new_key = calculateKey(cell);

            if (current.k1 < new_key.k1)
            {
                new_key.cell = cell;
                open_list_.push(new_key);
            }
            else if (getGValue(cell) > getRhsValue(cell))
            {
                g_values_[cell] = getRhsValue(cell);
                for (const auto &pred : getNeighbors(cell))
                {
                    updateVertex(pred);
                }
            }
            else
            {
                g_values_[cell] = std::numeric_limits<double>::infinity();
                updateVertex(cell);
                for (const auto &pred : getNeighbors(cell))
                {
                    updateVertex(pred);
                }
            }
        }
    }

    /**
     * getNeighbors()
     * Gets valid neighbor cells for a given cell
     */
    std::vector<CellIndex> VegmapPlanner::getNeighbors(const CellIndex &cell)
    {
        std::vector<CellIndex> neighbors;
        neighbors.reserve(8); // 8-connected grid

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                CellIndex neighbor = {cell.x + dx, cell.y + dy};

                // Check if within bounds and convert to unsigned int for bounds checking
                if (neighbor.x >= 0 &&
                    static_cast<unsigned int>(neighbor.x) < costmap_->getSizeInCellsX() &&
                    neighbor.y >= 0 &&
                    static_cast<unsigned int>(neighbor.y) < costmap_->getSizeInCellsY())
                {
                    neighbors.push_back(neighbor);
                }
            }
        }

        return neighbors;
    }

    /**
     * getCost()
     * Gets the cost to move from one cell to another
     */
    double VegmapPlanner::getCost(const CellIndex &from, const CellIndex &to)
    {
        // Make sure the coordinates are valid for getCost
        if (to.x < 0 || to.y < 0 ||
            static_cast<unsigned int>(to.x) >= costmap_->getSizeInCellsX() ||
            static_cast<unsigned int>(to.y) >= costmap_->getSizeInCellsY())
        {
            return std::numeric_limits<double>::infinity();
        }

        unsigned char cost = costmap_->getCost(static_cast<unsigned int>(to.x),
                                               static_cast<unsigned int>(to.y));

        if (cost >= lethal_cost_ && cost <= obstacle_range_)
        {
            return std::numeric_limits<double>::infinity();
        }

        // Calculate movement cost (diagonal movements cost more)
        double movement_cost = (from.x == to.x || from.y == to.y) ? 1.0 : std::sqrt(2.0);

        // Scale cost based on costmap
        double cost_multiplier = 1.0 + (static_cast<double>(cost) / 252.0);

        return movement_cost * cost_multiplier;
    }

    /**
     * extractPath()
     * Extracts the path from start to goal using the computed g-values
     */
    std::vector<CellIndex> VegmapPlanner::extractPath()
    {
        std::vector<CellIndex> path;
        CellIndex current = last_start_;

        // If start cell has infinite g-value, no path exists
        if (std::isinf(getGValue(current)))
        {
            return path;
        }

        path.push_back(current);

        // Follow lowest cost neighbor until goal is reached
        while (!(current == last_goal_))
        {
            double min_cost = std::numeric_limits<double>::infinity();
            CellIndex next_cell = current; // Default to current in case no better cell is found

            for (const auto &neighbor : getNeighbors(current))
            {
                double cost = getCost(current, neighbor);
                double total_cost = cost + getGValue(neighbor);

                if (total_cost < min_cost)
                {
                    min_cost = total_cost;
                    next_cell = neighbor;
                }
            }

            // If we can't make progress, break
            if (next_cell == current || std::isinf(min_cost))
            {
                break;
            }

            current = next_cell;
            path.push_back(current);
        }

        return path;
    }

    /**
     * worldToMap()
     * Converts world coordinates to map cell indices
     */
    bool VegmapPlanner::worldToMap(double wx, double wy, int &mx, int &my)
    {
        unsigned int umx, umy;
        bool result = costmap_->worldToMap(wx, wy, umx, umy);
        if (result)
        {
            mx = static_cast<int>(umx);
            my = static_cast<int>(umy);
        }
        return result;
    }

    /**
     * mapToWorld()
     * Converts map cell indices to world coordinates
     */
    void VegmapPlanner::mapToWorld(int mx, int my, double &wx, double &wy)
    {
        costmap_->mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
    }

    void VegmapPlanner::costmapUpdateCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {
        // Set the flag that costmap has changed
        costmap_changed_ = true;

        RCLCPP_DEBUG(
            node_->get_logger(), "Received costmap update notification");
    }

    /**
     * detectCostmapChanges()
     * Detects changes in the costmap since last planning
     */
    bool VegmapPlanner::detectCostmapChanges()
    {
        if (!costmap_received_)
        {
            // First time we receive the costmap, we assume no changes
            costmap_received_ = true;
            return false;
        }

        // Check if we've received a costmap update notification
        bool changed = costmap_changed_.exchange(false);

        if (changed)
        {
            RCLCPP_INFO(
                node_->get_logger(), "Costmap changed, will recompute path");
        }

        return changed;
    }

    /**
     * createPlan()
     * Method is called when planner server demands a global plan for specified start and goal pose.
     * @param start start pose
     * @param goal goal pose
     * @return nav_msgs::msg::Path carrying global plan
     */
    nav_msgs::msg::Path VegmapPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        RCLCPP_INFO(node_->get_logger(), "VegmapPlanner::createPlan called");
        RCLCPP_INFO(
            node_->get_logger(), "Received planning request from: %s to: %s",
            start.header.frame_id.c_str(), goal.header.frame_id.c_str());

        RCLCPP_INFO(
            node_->get_logger(), "Start: (%.2f, %.2f), Goal: (%.2f, %.2f)",
            start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

        nav_msgs::msg::Path global_path;

        // Checking if the goal and start state is in the global frame
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(
                node_->get_logger(), "Planner will only except start position from %s frame",
                global_frame_.c_str());
            return global_path;
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_INFO(
                node_->get_logger(), "Planner will only except goal position from %s frame",
                global_frame_.c_str());
            return global_path;
        }

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // Convert start and goal positions to grid cell coordinates
        int start_x, start_y, goal_x, goal_y;
        if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
            !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
        {
            RCLCPP_ERROR(
                node_->get_logger(), "Start or goal position outside of map bounds");
            return global_path;
        }

        CellIndex start_cell = {start_x, start_y};
        CellIndex goal_cell = {goal_x, goal_y};

        // Store current start and goal
        current_start_ = start;
        current_goal_ = goal;
        has_active_goal_ = true;

        // Check if we need to initialize or replan
        bool need_replan = false;

        if (last_start_.x < 0 || last_goal_.x < 0)
        {
            // First planning, initialize D* Lite
            reset();
            last_start_ = start_cell;
            last_goal_ = goal_cell;
            g_values_[goal_cell] = std::numeric_limits<double>::infinity();
            rhs_values_[goal_cell] = 0.0;
            updateVertex(goal_cell);
            need_replan = true;
        }
        else if (!(start_cell == last_start_) || !(goal_cell == last_goal_) || detectCostmapChanges())
        {
            // If start changed, goal changed, or costmap changed, update km and vertices
            if (!(start_cell == last_start_))
            {
                km_ += calculateHeuristic(last_start_, start_cell);
                last_start_ = start_cell;
            }

            // If goal changed, reset rhs values at goal
            if (!(goal_cell == last_goal_))
            {
                RCLCPP_INFO(
                    node_->get_logger(), "Goal changed, resetting planner");

                // Clear old goal
                rhs_values_[last_goal_] = std::numeric_limits<double>::infinity();
                g_values_[last_goal_] = std::numeric_limits<double>::infinity();

                // Set new goal
                last_goal_ = goal_cell;
                g_values_[goal_cell] = std::numeric_limits<double>::infinity();
                rhs_values_[goal_cell] = 0.0;
                updateVertex(goal_cell);
            }

            // For costmap changes, we'll just recompute the path
            need_replan = true;
        }

        if (need_replan)
        {
            computeShortestPath();
        }

        // Extract path
        auto path_cells = extractPath();

        if (path_cells.empty())
        {
            RCLCPP_WARN(
                node_->get_logger(), "Could not find a path from start to goal");
            return global_path;
        }

        // Convert cell path to world coordinates
        for (const auto &cell : path_cells)
        {
            geometry_msgs::msg::PoseStamped pose;

            mapToWorld(cell.x, cell.y, pose.pose.position.x, pose.pose.position.y);
            pose.pose.position.z = 0.0;

            // Calculate orientation based on path direction if not the last point
            if (&cell != &path_cells.back())
            {
                auto next_it = std::find(path_cells.begin(), path_cells.end(), cell);
                if (next_it != path_cells.end() && next_it + 1 != path_cells.end())
                {
                    auto next_cell = *(next_it + 1);
                    double dx = next_cell.x - cell.x;
                    double dy = next_cell.y - cell.y;

                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);

                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                }
                else
                {
                    // Fallback if we can't find the next cell
                    pose.pose.orientation = goal.pose.orientation;
                }
            }
            else
            {
                // Use goal orientation for the last point
                pose.pose.orientation = goal.pose.orientation;
            }

            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            global_path.poses.push_back(pose);
        }

        // If interpolation is requested, perform path smoothing
        if (interpolation_resolution_ > 0.0 && !global_path.poses.empty())
        {
            global_path = smoothPath(global_path);
        }

        return global_path;
    }

    /**
     * smoothPath()
     * Interpolates and smooths the path
     */
    nav_msgs::msg::Path VegmapPlanner::smoothPath(const nav_msgs::msg::Path &path)
    {
        nav_msgs::msg::Path smoothed_path;
        smoothed_path.header = path.header;

        if (path.poses.size() < 2)
        {
            return path;
        }

        // Add first pose
        smoothed_path.poses.push_back(path.poses.front());

        // Interpolate between consecutive poses
        for (size_t i = 0; i < path.poses.size() - 1; ++i)
        {
            const auto &p1 = path.poses[i].pose.position;
            const auto &p2 = path.poses[i + 1].pose.position;

            double distance = std::hypot(p2.x - p1.x, p2.y - p1.y);
            int steps = std::max(1, static_cast<int>(distance / interpolation_resolution_));

            for (int j = 1; j < steps; ++j)
            {
                double ratio = static_cast<double>(j) / steps;

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;

                // Interpolate position
                pose.pose.position.x = p1.x + ratio * (p2.x - p1.x);
                pose.pose.position.y = p1.y + ratio * (p2.y - p1.y);
                pose.pose.position.z = 0.0;

                // Interpolate orientation using slerp
                tf2::Quaternion q1, q2;
                tf2::fromMsg(path.poses[i].pose.orientation, q1);
                tf2::fromMsg(path.poses[i + 1].pose.orientation, q2);

                tf2::Quaternion q = tf2::slerp(q1, q2, ratio);
                pose.pose.orientation = tf2::toMsg(q);

                smoothed_path.poses.push_back(pose);
            }

            // Add the endpoint if it's not the last segment
            if (i < path.poses.size() - 2)
            {
                smoothed_path.poses.push_back(path.poses[i + 1]);
            }
        }

        // Add last pose
        smoothed_path.poses.push_back(path.poses.back());

        return smoothed_path;
    }

} // namespace vegmap_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vegmap_planner::VegmapPlanner, nav2_core::GlobalPlanner)