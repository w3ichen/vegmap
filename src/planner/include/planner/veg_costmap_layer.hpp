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
#include "planner_msgs/srv/get_transforms.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define UNKNOWN_COST 120
#define UNKNOWN_STDDEV 20

namespace veg_costmap
{
    class VegCostmapLayer : public nav2_costmap_2d::CostmapLayer
    {
    public:
        // Constructor
        VegCostmapLayer();

        // Define a struct to store zone information
        struct ZoneInfo
        {
            double x;
            double y;
            double radius;
            std::string name;
        };

        // Define ObstaclePoint first, before using it in containers
        struct ObstaclePoint
        {
            // World coords from gazebo are floats
            float x{std::numeric_limits<float>::max()};
            float y{std::numeric_limits<float>::max()};
            // Costmap coords are in ints
            uint16_t mx{0};
            uint16_t my{0};
            std::string name{""};
            mutable unsigned char cost{0};
            mutable unsigned char cost_stddev{0};

            // Default constructor
            ObstaclePoint() = default;

            // Custom equality operator that only compares x and y
            bool operator==(const ObstaclePoint &other) const
            {
                return (mx == other.mx && my == other.my);
            }
        };

        // Required for std::unordered_set to work with ObstaclePoint
        struct ObstaclePointHash
        {
            std::size_t operator()(const ObstaclePoint &p) const
            {
                std::size_t h1 = std::hash<uint16_t>{}(p.mx);
                std::size_t h2 = std::hash<uint16_t>{}(p.my);
                return h1 ^ (h2 << 1);
            }
        };

        struct ObstacleData
        {
            // Use the custom hash function in the unordered_set definition
            std::unordered_set<ObstaclePoint, ObstaclePointHash> others;
            unsigned char cost{UNKNOWN_COST};          // Default cost
            unsigned char cost_stddev{UNKNOWN_STDDEV}; // Default stddev
        };

        // Required costmap layer methods
        virtual void onInitialize() override;
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y) override;
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j) override;
        virtual void reset() override;
        virtual bool isClearable() override { return true; }

    private:
        // Use Layer's parameter methods
        using nav2_costmap_2d::Layer::declareParameter;

        // Obstacles considered as vegetation
        std::vector<std::string> veg_names_ = {"tree", "bush", "grass", "rock"};
        bool getWorldTransforms(void);

        // Callbacks
        void updateCostCallback(
            const std::shared_ptr<planner_msgs::srv::UpdateCost::Request> request,
            std::shared_ptr<planner_msgs::srv::UpdateCost::Response> response);
        bool updateCostValue_(nav2_costmap_2d::Costmap2D *costmap, unsigned int mx, unsigned int my, unsigned char cost);
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
        bool enabled_;
        std::mutex mutex_;
        bool costmap_updated_{false};
        std::string world_tf_service_;
        std::string update_topic_;
        std::string updated_topic_;
        std::string costmap_pub_topic_;
        std::string global_frame_;
        double obstacle_range_;
        double obstacle_radius_;
        double tree_radius_;

        int lethal_cost_;
        bool use_gradient_costs_;
        double gradient_factor_;
        unsigned int map_width_;
        unsigned int map_height_;
        double map_resolution_;
        double map_origin_x_;
        double map_origin_y_;
        std::string world_name_;

        // Random generators
        std::mt19937 gen;

        // Helper Functions
        void bayesianUpdateGaussian(
            double prior_mean, double prior_stddev,
            double obs_mean, double obs_stddev,
            unsigned char *posterior_mean, unsigned char *posterior_stddev);
        double clampCost(double cost);
        void fetchObstacleDB(std::string obstacle_name, ObstacleData **obstacle_data);
    };

    namespace defaults
    {
        // Prior knowledge base of vegetation obstacles
        static const std::initializer_list<std::pair<const std::string, VegCostmapLayer::ObstacleData>> SAVED_OBSTACLE_DATABASE = {
            // {"grass", {{}, 20, 15}},
            // {"bush", {{}, 180, 10}},
            // {"tree", {{}, 250, 2}},
            // {"rock", {{}, 250, 2}},

            // // Lethal costs
            {"grass", {{}, 252, 0}},
            {"bush", {{}, 252, 0}},
            {"tree", {{}, 252, 0}},
            {"rock", {{}, 252, 0}},
        };

    } // namespace defaults

} // namespace veg_costmap

#endif // VEG_COSTMAP_LAYER_HPP