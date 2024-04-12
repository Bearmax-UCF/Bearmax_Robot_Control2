#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <unistd.h>

#include "bearmax_msgs/action/task.hpp"

#include "bearmax_moveit/task_server.hpp"

const std::string PLANNING_GROUP = "all_group";

MoveitTaskServer::MoveitTaskServer()
    : Node("moveit_tasks_node",
            rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true))
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Task>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "task",
            std::bind(&MoveitTaskServer::handle_goal, this, _1, _2),
            std::bind(&MoveitTaskServer::handle_cancel, this, _1),
            std::bind(&MoveitTaskServer::handle_accepted, this, _1)
            );

    speech_pub_ = this->create_publisher<std_msgs::msg::String>("speech", 10);

    // CHANGE THIS PLEASE RAAHYM
    head_sub_ = this->create_subscription<geometry_msgs::msg::Point>("head_in",
        1, std::bind(&MoveitTaskServer::face_follower, this, _1));

    foot_z = this->get_parameter("foot_z").as_double();

    head_position_error = this->get_parameter("head_position_error").as_double();

    // Register task executors,
    // task executor functions must be named execute_<task_name>

    REGISTER_TASK(test);
    REGISTER_TASK(neutral);
    REGISTER_TASK(happy);
    REGISTER_TASK(sad);
    REGISTER_TASK(angry);
    REGISTER_TASK(confused);
    REGISTER_TASK(shocked);
    REGISTER_TASK(worried);
    REGISTER_TASK(scared);
    REGISTER_TASK(annoyed);
}

/* ========== Task Executors ========== */

void MoveitTaskServer::execute_test(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Test Pose Not for full implementation");

        auto testState = JointValueMap{
            {L_ARM_ROTATOR, (0.000)},
            {L_ARM_GRIP, {1.500}}
        };

        move_group_->setJointValueTarget(testState);
        move_group_->move();

        RCLCPP_INFO(this->get_logger(),
                "Test pose assumed");

        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(test);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }

    }

void MoveitTaskServer::execute_neutral(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "RETURNING TO NEUTRAL POSITION");

        
        
        auto restState = JointValueMap{
            {CHASSIS, {0.000}},
            {L_ARM_GRIP, (0.000)},
            {L_ARM_SHOULDER, (0.000)},
            {L_ARM_ROTATOR, (0.000)},
            {L_ARM_ELBOW, (0.000)},
            {L_ARM_PAW, (0.000)},
            {R_ARM_GRIP, (0.000)},
            {R_ARM_SHOULDER, (0.000)},
            {R_ARM_ROTATOR, (0.000)},
            {R_ARM_ELBOW, (0.000)},
            {R_ARM_PAW, (0.000)},
            {L_HEAD, (0.000)},
            {R_HEAD, (0.000)},
            {HEAD_YAW, (0.000)}
        };

        /*
        auto restState = JointValueMap{
            {CHASSIS, {0.000}},
            {L_ARM_SHOULDER, (3.141)},
            {L_ARM_ROTATOR, (1.505)},
            {L_ARM_ELBOW, (0.000)},
            {L_ARM_GRIP, (1.505)},
            {L_ARM_THUMB, (0.000)},
            {R_ARM_SHOULDER, (0.000)},
            {R_ARM_ROTATOR, (1.505)},
            {R_ARM_ELBOW, (3.141)},
            {L_ARM_PAW, (1.505)},
            {R_ARM_GRIP, (0.000)},
            {R_ARM_PAW, (0.000)},
            {HEAD_YAW, (1.505)},
            {L_HEAD, (1.505)},
            {R_HEAD, (1.505)}
        };*/


        move_group_->setJointValueTarget(restState);
        move_group_->move();

        RCLCPP_INFO(this->get_logger(),
                "Neutral pose assumed");

        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(neutral);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
    }


void MoveitTaskServer::execute_happy(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: happy");

        // Complex Emotion
        /*
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap{
                {L_ARM_SHOULDER, (-2.445)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_GRIP, (1.505)},
                {R_ARM_SHOULDER, (2.445)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_GRIP, (-1.505)}
            };

            tlst.emplace_back(stateOne);

            auto stateTwo = JointValueMap{
                {L_ARM_SHOULDER, (-1.445)},
                {L_ARM_ELBOW, (1.222)},
                {R_ARM_SHOULDER, (1.445)},
                {R_ARM_ELBOW, (1.222)}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);

            auto stateThree = JointValueMap {
                {L_ARM_SHOULDER, (-2.445)},
                {L_ARM_ELBOW, (0.000)},
                {R_ARM_SHOULDER, (2.445)},
                {R_ARM_ELBOW, (0.000)}
            };
            
            tlst.emplace_back(stateThree);

            auto stateFour = JointValueMap {
                {L_ARM_ROTATOR, (-1.409)},
                {R_ARM_ROTATOR, (-1.409)}
            };

            tlst.emplace_back(stateFour);
            

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            tlst.emplace_back(restState);

            return tlst;
        }();*/

        // Simple Emotion
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;
            
            auto stateOne = JointValueMap{
                {L_ARM_SHOULDER, (-2.445)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_GRIP, (1.505)},
                {R_ARM_SHOULDER, (2.445)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_GRIP, (-1.505)}
            };

            tlst.emplace_back(stateOne);

            auto stateTwo = JointValueMap{
                {L_ARM_SHOULDER, (-1.445)},
                {L_ARM_ELBOW, (1.222)},
                {R_ARM_SHOULDER, (1.445)},
                {R_ARM_ELBOW, (1.222)}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);


            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            //tlst.emplace_back(restState);

            return tlst;
        }();


        RCLCPP_INFO(this->get_logger(), "WOOOOOOOOO");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "Annnnd it's gone.");

        
        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(happy);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
    }

void MoveitTaskServer::execute_sad(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: sad");

        // Complex Emotion
        /*
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap{
                {L_ARM_GRIP, (1.573)},
                {L_ARM_SHOULDER, (-1.834)},
                {L_ARM_ROTATOR, (-0.51)},
                {L_ARM_ELBOW, (1.222)},
                {R_ARM_GRIP, (-1.573)},
                {R_ARM_SHOULDER, (1.936)},
                {R_ARM_ROTATOR, (-0.73)},
                {R_ARM_ELBOW, (1.222)}
            };

            tlst.emplace_back(stateOne);
            
            auto stateTwo = JointValueMap{
                {L_ARM_GRIP, (1.573)},
                {L_ARM_SHOULDER, (-1.834)},
                {L_ARM_ROTATOR, (-0.400)},
                {L_ARM_ELBOW, (1.222)},
                {R_ARM_GRIP, (-1.573)},
                {R_ARM_SHOULDER, (1.936)},
                {R_ARM_ROTATOR, (-0.600)},
                {R_ARM_ELBOW, (1.222)}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);


            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            tlst.emplace_back(restState);

            return tlst;
        }();*/

        // Simple Emotion
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto simpleState = JointValueMap{    
                {L_ARM_SHOULDER, (-1.834)},
                {L_ARM_ROTATOR, (-0.51)},
                {L_ARM_ELBOW, (1.222)},
                {L_ARM_GRIP, (1.573)},
                {R_ARM_SHOULDER, (1.936)},
                {R_ARM_ROTATOR, (-0.73)},
                {R_ARM_ELBOW, (1.222)},
                {R_ARM_GRIP, (-1.573)}
            };
            
            tlst.emplace_back(simpleState);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            //tlst.emplace_back(restState);

            return tlst;
        }();        

        RCLCPP_INFO(this->get_logger(), "Staying sad until canceled...");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "Locking in to normal");


        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(sad);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }

    }

void MoveitTaskServer::execute_angry(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: angry");

        // Complex Movement
        /*
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap{
                {R_ARM_SHOULDER, (1.717)},
                {R_ARM_ELBOW, (1.500)}
            };

            tlst.emplace_back(stateOne);
            
            auto stateTwo = JointValueMap{
                {R_ARM_SHOULDER, (1.717)},
                {R_ARM_ELBOW, (0.000)}
            };

            

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);

            auto stateThree = JointValueMap{
                {CHASSIS, (-0.400)},
                {L_ARM_SHOULDER, (-1.391)},
                {L_ARM_ELBOW, (1.118)},
                {R_ARM_SHOULDER, (1.391)},
                {R_ARM_ELBOW, (1.118)},
                {HEAD_YAW, (0.250)}
            };

            tlst.emplace_back(stateThree);

            auto stateFour = JointValueMap{
                {L_ARM_ROTATOR, (-0.441)},
                {L_ARM_GRIP, (1.573)},
                {R_ARM_ROTATOR, (-0.441)},
                {R_ARM_GRIP, (-1.573)}
            };

            tlst.emplace_back(stateFour);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };
            tlst.emplace_back(restState);

            return tlst;
        }();*/

        // Simple Movement for test
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

             auto simpleState = JointValueMap{
                {CHASSIS, (-0.400)},
                {L_ARM_SHOULDER, (-1.391)},
                {L_ARM_ROTATOR, (-0.441)},
                {L_ARM_ELBOW, (1.118)},
                {L_ARM_GRIP, (1.573)},
                {L_ARM_PAW, (1.545)},
                {R_ARM_SHOULDER, (1.391)},
                {R_ARM_ROTATOR, (-0.441)},
                {R_ARM_ELBOW, (1.118)},
                {R_ARM_GRIP, (-1.573)},
                {R_ARM_PAW, (1.545)},
                {HEAD_YAW, (0.250)}
            };

            tlst.emplace_back(simpleState);
            

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };
            
            //tlst.emplace_back(restState);

            return tlst;
        }();

        RCLCPP_INFO(this->get_logger(), "grrrr...");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "Locking in to normal");

        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(angry);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }

    }

void MoveitTaskServer::execute_confused(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: confused");

        // Complex Emotion Definition
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap { // ADD RIGHT PAW MOVEMENT & NECK MOVEMENT IN HARDWARE IMPLEMENTATION
                {R_ARM_SHOULDER, (2.717)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_PAW, {0.000}}
                //{R_HEAD, (0.0872)}
            };

            tlst.emplace_back(stateOne);

            auto stateTwo = JointValueMap { // ADD RIGHT PAW MOVEMENT & NECK MOVEMENT IN HARDWARE IMPLEMENTATION
                {R_ARM_SHOULDER, (2.717)},
                {R_ARM_GRIP, (-0.774)},
                {R_ARM_PAW, {0.500}}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            tlst.emplace_back(restState);

            return tlst;
        }();

        RCLCPP_INFO(this->get_logger(), "Staying confused until someone explains what's going on");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "Ahh I get it now");

        
        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(confused);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
    }

void MoveitTaskServer::execute_shocked(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: shocked");


        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap {
                {CHASSIS, {0.600}}
            };

            tlst.emplace_back(stateOne);


            auto stateTwo = JointValueMap {
                {CHASSIS, {0.000}},
                {L_ARM_SHOULDER, (-2.615)},
                {R_ARM_SHOULDER, (2.615)}
            };

            tlst.emplace_back(stateTwo);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            tlst.emplace_back(restState);
            
            return tlst;
        }();

        RCLCPP_INFO(this->get_logger(), "EGAD HOW CAN THIS BE");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "It is what it is");

        
        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(shocked);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
    }

void MoveitTaskServer::execute_worried(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: worried");
        
        // Complex Emotion ADD NECK MOVEMENT WHEN FULLY TESTING
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap {
                {L_ARM_SHOULDER, {-1.151}},
                {L_ARM_ROTATOR, {-0.56}},
                {L_ARM_ELBOW, {1.154}},
                {R_ARM_SHOULDER, {1.657}}, //SUBJECT TO CHANGE
                {R_ARM_ELBOW, {0.934}},
                {R_ARM_PAW, {0.000}}
            };

            tlst.emplace_back(stateOne);
            
            // IN FINAL VERSION CHANGE THIS SO THAT THE PAW MOVES NOT ROTATOR
            auto stateTwo = JointValueMap {
                {L_ARM_SHOULDER, {-1.151}},
                {L_ARM_ROTATOR, {-0.56}},
                {L_ARM_ELBOW, {1.154}},
                {R_ARM_SHOULDER, {1.657}}, //SUBJECT TO CHANGE
                {R_ARM_ELBOW, {0.934}},
                {R_ARM_PAW, {1.154}}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            tlst.emplace_back(restState);

            return tlst;


        }();

        RCLCPP_INFO(this->get_logger(), "ERM what the flip?");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "Moving back to normal");


        

        
        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(worried);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }

    }

void MoveitTaskServer::execute_scared(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: scared");

        // Complex emotion
        /*
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap {
                {L_ARM_SHOULDER, (-1.834)},
                {L_ARM_ROTATOR, (-0.51)},
                {L_ARM_ELBOW, (1.222)},
                {L_ARM_GRIP, (1.573)},
                {R_ARM_SHOULDER, (1.936)},
                {R_ARM_ROTATOR, (-0.73)},
                {R_ARM_ELBOW, (1.222)},
                {R_ARM_GRIP, (-1.573)},
            };

            tlst.emplace_back(stateOne);

            auto stateTwo = JointValueMap {
                {CHASSIS, (-0.977)},
                {L_ARM_SHOULDER, (-1.494)},
                {L_ARM_ROTATOR, (-0.934)},
                {L_ARM_ELBOW, (1.307)},
                {L_ARM_GRIP, (0.00)},
                {R_ARM_SHOULDER, (1.494)},
                {R_ARM_ROTATOR, (-0.934)},
                {R_ARM_ELBOW, (1.46)},
                {R_ARM_GRIP, (0.00)},
                {HEAD_YAW, (0.628)}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            tlst.emplace_back(restState);

            return tlst;


        }();
        */
        // Simple emotion for test
        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto simpleState = JointValueMap {
                {CHASSIS, (-0.977)},
                {L_ARM_SHOULDER, (-1.494)},
                {L_ARM_ROTATOR, (-0.934)},
                {L_ARM_ELBOW, (1.307)},
                {L_ARM_GRIP, (1.573)},
                {R_ARM_SHOULDER, (1.494)},
                {R_ARM_ROTATOR, (-0.934)},
                {R_ARM_ELBOW, (1.46)},
                {R_ARM_GRIP, (-1.573)},
                {HEAD_YAW, (0.628)}
            };

            tlst.emplace_back(simpleState);

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            //tlst.emplace_back(restState);

            return tlst;


        }();

        RCLCPP_INFO(this->get_logger(), "Is it getting cold in here? or is it me...");

        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "We're locking in now");

        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(scared);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }


    }
    

void MoveitTaskServer::execute_annoyed(
    const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal: annoyed");
        

        auto const target_list = [this]{
            std::vector<JointValueMap> tlst;

            auto stateOne = JointValueMap {
                {CHASSIS, (0.400)},
                {L_ARM_SHOULDER, (-1.155)},
                {L_ARM_ROTATOR, (-1.053)},
                {L_ARM_ELBOW, (1.545)},
                {R_ARM_SHOULDER, (0.645)},
                {R_ARM_ROTATOR, (-1.053)},
                {R_ARM_ELBOW, (1.545)},
                {HEAD_YAW, (0.56)} 
            };
            
            tlst.emplace_back(stateOne);

            auto stateTwo = JointValueMap {
                {CHASSIS, (0.400)},
                {L_ARM_SHOULDER, (-1.155)},
                {L_ARM_ROTATOR, (-1.053)},
                {L_ARM_ELBOW, (1.545)},
                {R_ARM_SHOULDER, (0.645)},
                {R_ARM_ROTATOR, (-1.053)},
                {R_ARM_ELBOW, (1.545)},
                {HEAD_YAW, (0.00)}
            };

            tlst.emplace_back(stateTwo);
            tlst.emplace_back(stateOne);
            tlst.emplace_back(stateTwo);
            

            auto restState = JointValueMap{
                {CHASSIS, {0.000}},
                {L_ARM_GRIP, (0.000)},
                {L_ARM_SHOULDER, (0.000)},
                {L_ARM_ROTATOR, (0.000)},
                {L_ARM_ELBOW, (0.000)},
                {L_ARM_PAW, (0.000)},
                {R_ARM_GRIP, (0.000)},
                {R_ARM_SHOULDER, (0.000)},
                {R_ARM_ROTATOR, (0.000)},
                {R_ARM_ELBOW, (0.000)},
                {R_ARM_PAW, (0.000)},
                {L_HEAD, (0.000)},
                {R_HEAD, (0.000)},
                {HEAD_YAW, (0.000)}
            };

            //tlst.emplace_back(restState);
            return tlst;

        }();

        RCLCPP_INFO(this->get_logger(), "HMPH...Not talking to you...");
        
        int move_cnt = 1;
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(), "Running move frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);
            move_group_->move();

            RCLCPP_INFO(this->get_logger(), "Finished move frame: %d", move_cnt);

            move_cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "I'm not annoyed anymore");

        auto result = std::make_shared<Task::Result>();
        result->success = true;
        //RESUME_FACE_FOLLOWER(annoyed);
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }


    }




/* ==================================== */

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitTaskServer>();

    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
    node->setup_moveit(&move_group_interface);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
