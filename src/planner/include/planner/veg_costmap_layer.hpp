#ifndef VEG_COSTMAP_LAYER_HPP
#define VEG_COSTMAP_LAYER_HPP

#include <string>
#include <mutex>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/empty.hpp"

namespace veg_costmap
{

    class VegCostmapLayer : public nav2_costmap_2d::CostmapLayer
    {
    public:
        VegCostmapLayer();

        virtual void onInitialize() override;
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y) override;
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j) override;

        virtual void reset() override { obstacle_points_.clear(); }
        virtual bool isClearable() override { return true; }

        // Method to update the costmap from an external source
        void updateVegetationCost(unsigned int mx, unsigned int my, unsigned char cost);

    private:
        void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        struct ObstaclePoint
        {
            double x;
            double y;
            std::string name;
            unsigned char cost; // Allow variable cost based on vegetation type
        };

        std::vector<ObstaclePoint> obstacle_points_;
        std::mutex mutex_;
        bool costmap_updated_;

        // Subscription for TF messages
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

        // Subscription for external costmap updates
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

        // Service to trigger replanning
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;

        // Parameters
        std::string tf_topic_;
        std::string veg_update_topic_;
        std::string global_frame_;
        double obstacle_range_;
        double obstacle_radius_;
        bool clearing_enabled_;
        std::string tf_prefix_filter_;
        unsigned char lethal_cost_;
        unsigned char medium_cost_;
        unsigned char low_cost_;
        bool use_gradient_costs_;
        double gradient_factor_;
    };

    double distance(double x1, double y1, double x2, double y2);

} // namespace veg_costmap

#endif // VEG_COSTMAP_LAYER_HPP