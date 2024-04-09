#ifndef BEARMAX_MOVEIT__TASK_SERVER_HPP_
#define BEARMAX_MOVEIT__TASK_SERVER_HPP_
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "bearmax_msgs/action/task.hpp"

#define REGISTER_TASK(task_name) tasks_.emplace(\
        #task_name,\
        std::bind(&MoveitTaskServer::execute_##task_name, this, _1)\
        )
#define DEFINE_TASK_EXECUTOR(task_name) \
    void execute_##task_name(\
        const std::shared_ptr<GoalHandleTask> goal_handle)

#define RESUME_FACE_FOLLOWER(task_name) \
    should_follow = true;\
    RCLCPP_INFO(this->get_logger(), "[%s]: Resumed Face Follower", #task_name)
#define PI 3.1416

// Define Joint Names

const std::string CHASSIS = "chassis_joint";

// Left arm & hand
const std::string L_ARM_SHOULDER = "left_shoulder_joint";
const std::string L_ARM_ROTATOR = "left_rotator_joint";
const std::string L_ARM_ELBOW = "left_elbow_joint";
const std::string L_ARM_GRIP = "left_grip_joint";
const std::string L_ARM_THUMB = "left_thumb_joint";
const std::string L_ARM_PAW = "left_paw_joint";

// Right arm & hand
const std::string R_ARM_SHOULDER = "right_shoulder_joint";
const std::string R_ARM_ROTATOR = "right_rotator_joint";
const std::string R_ARM_ELBOW = "right_elbow_joint";
const std::string R_ARM_GRIP =  "right_grip_joint";
const std::string R_ARM_THUMB = "right_thumb_joint";
const std::string R_ARM_PAW = "right_paw_joint";

// head
const std::string L_HEAD = "left_neck_joint";
const std::string R_HEAD = "right_neck_joint";
const std::string HEAD_YAW = "neck_yaw_joint";


class MoveitTaskServer : public rclcpp::Node
{
    public:
        using Task = bearmax_msgs::action::Task;
        using GoalHandleTask = rclcpp_action::ServerGoalHandle<Task>;
        using JointValueMap = std::map<std::string, double>;

        MoveitTaskServer();

        void setup_moveit(
            moveit::planning_interface::MoveGroupInterface *move_group)
        {
            move_group_ = move_group;
        }
    private:
        using task_executor = std::function<void(const std::shared_ptr<GoalHandleTask> goal_handle)>;
        std::map<std::string, task_executor> tasks_;
        rclcpp_action::Server<Task>::SharedPtr action_server_;
        moveit::planning_interface::MoveGroupInterface *move_group_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr head_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        bool should_follow = true;
        // face_follower_task variables
        double foot_z = 1.0;
        double last_x = 0;
        double last_y = 0;
        double last_z = 0;
        bool is_quizzical = false;
        double head_position_error = 0.02;
        // FIXME: Get current state, don't assume start is always 0
        JointValueMap last_state;
        double last_yaw = 0;
        double last_pitch = 0;
        /////

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Task::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(),
                    "Received goal request with task %s",
                    goal->task_name.c_str());
            (void)uuid;
            // Reject task goal if no executor exists for it.
            if (!tasks_.contains(goal->task_name)) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            should_follow = false;
            RCLCPP_INFO(this->get_logger(),
                    "Paused Face Follower");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleTask> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(),
                    "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void execute(const std::shared_ptr<GoalHandleTask> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Running Task Executor...");

            const auto goal = goal_handle->get_goal();
            const auto task_name = goal->task_name;

            auto cb = tasks_.at(task_name);

            cb(goal_handle);
        }

        void handle_accepted(
            const std::shared_ptr<GoalHandleTask> goal_handle)
        {
            using namespace std::placeholders;
            RCLCPP_INFO(this->get_logger(), "Accepted Goal, executing...");

            // this needs to return quickly to avoid blocking executor,
            // so spin up a new thread.
            std::thread{
                std::bind(&MoveitTaskServer::execute, this, _1),
                    goal_handle
            }.detach();
        }

        void face_follower(const geometry_msgs::msg::Point & msg);

        /********** NEW Task Executors **********/
        DEFINE_TASK_EXECUTOR(test);
        DEFINE_TASK_EXECUTOR(neutral);
        DEFINE_TASK_EXECUTOR(happy);
        DEFINE_TASK_EXECUTOR(sad);
        DEFINE_TASK_EXECUTOR(angry);
        DEFINE_TASK_EXECUTOR(confused);
        DEFINE_TASK_EXECUTOR(shocked);
        DEFINE_TASK_EXECUTOR(worried);
        DEFINE_TASK_EXECUTOR(scared);
        DEFINE_TASK_EXECUTOR(annoyed);
};

#endif
