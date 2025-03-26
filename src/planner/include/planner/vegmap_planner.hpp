#ifndef VEGMAP_PLANNER_HPP_
#define VEGMAP_PLANNER_HPP_

#include <string>
#include <memory>
#include <queue>
#include <vector>
#include <unordered_map>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2/utils.h"
#include "std_msgs/msg/empty.hpp"

namespace vegmap_planner
{

    struct CellIndex
    {
        int x;
        int y;

        bool operator==(const CellIndex &other) const
        {
            return x == other.x && y == other.y;
        }
    };

    struct CellIndexHash
    {
        std::size_t operator()(const CellIndex &idx) const
        {
            return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
        }
    };

    struct StateKey
    {
        double k1;
        double k2;
        CellIndex cell;

        bool operator<(const StateKey &other) const
        {
            if (k1 == other.k1)
                return k2 > other.k2;
            return k1 > other.k1;
        }
    };

    class VegmapPlanner : public nav2_core::GlobalPlanner
    {
    public:
        VegmapPlanner() = default;
        ~VegmapPlanner() = default;

        // Plugin methods from GlobalPlanner
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    private:
        // Costmap update callback
        void costmapUpdateCallback(const std_msgs::msg::Empty::SharedPtr msg);

        // D* Lite algorithm methods
        void reset();
        double calculateHeuristic(const CellIndex &a, const CellIndex &b);
        StateKey calculateKey(const CellIndex &cell);
        double getGValue(const CellIndex &cell);
        double getRhsValue(const CellIndex &cell);
        void updateVertex(const CellIndex &cell);
        void computeShortestPath();
        std::vector<CellIndex> getNeighbors(const CellIndex &cell);
        double getCost(const CellIndex &from, const CellIndex &to);
        std::vector<CellIndex> extractPath();

        // Utility methods
        bool worldToMap(double wx, double wy, int &mx, int &my);
        void mapToWorld(int mx, int my, double &wx, double &wy);
        bool detectCostmapChanges();
        nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path &path);

        // Node and costmap references
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::string global_frame_;
        std::string name_;

        // Costmap change detection
        std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr costmap_update_sub_;
        std::atomic<bool> costmap_changed_{false};
        bool costmap_received_{false};

        // Current plan information
        geometry_msgs::msg::PoseStamped current_start_;
        geometry_msgs::msg::PoseStamped current_goal_;
        bool has_active_goal_{false};

        // D* Lite parameters
        double interpolation_resolution_{0.1};
        double heuristic_weight_{1.0};
        unsigned char lethal_cost_{253};
        unsigned char obstacle_range_{254};

        // D* Lite data structures
        std::unordered_map<CellIndex, double, CellIndexHash> g_values_;
        std::unordered_map<CellIndex, double, CellIndexHash> rhs_values_;
        std::priority_queue<StateKey> open_list_;
        double km_{0.0};
        CellIndex last_start_;
        CellIndex last_goal_;
    };

} // namespace vegmap_planner

#endif // VEGMAP_PLANNER_HPP_