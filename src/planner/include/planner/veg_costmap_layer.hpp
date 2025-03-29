#ifndef VEG_COSTMAP_LAYER_HPP
#define VEG_COSTMAP_LAYER_HPP

#include <string>
#include <mutex>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/empty.hpp"
#include "planner_msgs/srv/update_cost.hpp"

namespace veg_costmap
{
    class VegCostmapLayer : public nav2_costmap_2d::CostmapLayer
    {
    public:
        // Constructor
        VegCostmapLayer();

        // Required costmap layer methods
        virtual void onInitialize() override;
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y) override;
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j) override;
        virtual void matchSize() override;
        virtual void onFootprintChanged(void) override;
        virtual void reset() override;
        virtual bool isClearable() override { return true; }

        // Obstacle
        double getSavedObstacleCost(std::string obstacle_name);
        struct ObstaclePoint
        {
            double x;
            double y;
            std::string name;
            unsigned char cost; // Allow variable cost based on vegetation type
            unsigned char cost_stddev;
        };
        struct ObstacleData
        {
            std::unordered_set<ObstaclePoint> others; // List of other similar obstacles
            unsigned char cost;                       // Allow variable cost based on vegetation type
            unsigned char cost_stddev;
        };

    private:
        // Obstacles considered as vegetation
        std::vector<std::string> veg_names_ = {"tree", "bush"};
        bool getWorldTransforms(void);

        // Callbacks
        void updateCostCallback(
            const std::shared_ptr<planner_msgs::srv::UpdateCost::Request> request,
            std::shared_ptr<planner_msgs::srv::UpdateCost::Response> response);
        bool VegCostmapLayer::updateCostValue_(nav2_costmap_2d::Costmap2D *costmap, unsigned int mx, unsigned int my, unsigned char cost);
        void publishCostmapCallback();

        // Data structures
        std::vector<std::vector<ObstaclePoint>> obstacle_grid_;
        std::unordered_map<std::string, ObstacleData> obstacle_database_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

        // Services
        rclcpp::Service<planner_msgs::srv::UpdateCost>::SharedPtr update_cost_srv_;

        // Timers
        rclcpp::TimerBase::SharedPtr costmap_pub_timer_;

        // Parameters
        std::mutex mutex_;
        bool costmap_updated_;
        std::string world_tf_service_;
        std::string update_topic_;
        std::string updated_topic_;
        std::string costmap_pub_topic_;
        std::string global_frame_;
        double obstacle_range_;
        double obstacle_radius_;
        int lethal_cost_;
        bool use_gradient_costs_;
        double gradient_factor_;
        int map_width_;
        int map_height_;
        double map_resolution_;
        double map_origin_x_;
        double map_origin_y_;

        // Random generators
        std::mt19937 gen;
    };

    namespace defaults
    {
        // Prior knowledge base of vegetation obstacles
        static const std::initializer_list<std::pair<const std::string, VegCostmapLayer::ObstacleData>> SAVED_OBSTACLE_DATABASE = {
            {"tree_1", {{}, 250, 2}},
            {"tree_2", {{}, 250, 2}},
            {"tree_3", {{}, 250, 2}},
            {"tree_4", {{}, 250, 2}},
            {"tree_5", {{}, 250, 2}},
            {"tree_6", {{}, 250, 2}},
            {"tree_7", {{}, 250, 2}},
            {"tree_8", {{}, 250, 2}},
            {"palm_tree", {{}, 254, 2}},
            {"bush_1", {{}, 150, 10}},
            {"bush_2", {{}, 150, 10}},
            {"bush_3", {{}, 150, 10}},
            {"bush_4", {{}, 150, 10}},
        };

        uint8_t UNKNOWN_COST = 120;
        uint8_t UNKNOWN_COVARIANCE = 20;
    } // namespace defaults

} // namespace veg_costmap

#endif // VEG_COSTMAP_LAYER_HPP