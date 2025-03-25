
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp" // https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action

using NavToPoseAction = nav2_msgs::action::NavigateToPose;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<NavToPoseAction>;

class PlannerServer : public rclcpp::Node
{
public:
    explicit PlannerServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("planner_server", options)
    {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<NavToPoseAction>(
            this,
            "move_robot",
            std::bind(&PlannerServer::handle_goal, this, _1, _2),
            std::bind(&PlannerServer::handle_cancel, this, _1),
            std::bind(&PlannerServer::handle_accepted, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    rclcpp_action::Server<NavToPoseAction>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavToPoseAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: (%f, %f)", goal->pose.pose.position.x, goal->pose.pose.position.y);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&PlannerServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(20);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavToPoseAction::Feedback>();

        auto result = std::make_shared<NavToPoseAction::Result>();
        auto move = geometry_msgs::msg::Twist();
        for (int i = 0; i < 5 && rclcpp::ok(); ++i)
        {
            // Check if there is a cancel request
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Send move command to the robot
            move.linear.x = 1;
            move.angular.z = 0.1;
            publisher_->publish(move);

            // Send feedback
            feedback->current_pose = goal->pose;                         // Example feedback
            feedback->navigation_time = rclcpp::Duration(1, 0);          // Example feedback
            feedback->estimated_time_remaining = rclcpp::Duration(1, 0); // Example feedback
            feedback->number_of_recoveries = 0;                          // Example feedback
            feedback->distance_remaining = 5 - i;                        // Example feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Feedback published");

            loop_rate.sleep();
        }
        // Check if goal is done
        if (rclcpp::ok())
        {
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<PlannerServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}