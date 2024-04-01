#include "bearmax_moveit/task_server.hpp"
#include <cmath>

void MoveitTaskServer::face_follower(const geometry_msgs::msg::Point & msg)
{
    if (!should_follow) {
        return;
    }
    if (msg.x == last_x && msg.y == last_y && msg.z == last_z) {
        return;
    }
    // Store previous x, y, & z values
    last_x = msg.x;
    last_y = msg.y;
    last_z = msg.z;

    // Read new x, y, & z values
    RCLCPP_INFO(this->get_logger(), "Head POS: (%f, %f, %f)",
            msg.x, msg.y, msg.z);

    // Terrible distance value
    double dist = (1 - msg.z) / (foot_z * 2.0);

    // x- to right; x+ to left
    double delta_x = msg.x - 0.5;
    // y- to down; y+ to up
    double delta_y = 0.5 - msg.y;

    // Don't update head position if we're only off +-head_position_error
    if (abs(delta_x) <= head_position_error &&
            abs(delta_y) <= head_position_error) {
        RCLCPP_INFO(this->get_logger(),
                "Head did not move enough! Not changing position.");
        return;
    }


    // Set target Joint States
    auto const target_joints = [&delta_x, &delta_y, &dist, this]{
        std::map<std::string, double> vals;

        // These values are in radians
        //                vals[HEAD_ROLL] = PI / 3.0;
        double dyaw = asin(delta_x / dist);
        if (abs(dyaw + last_yaw) >= (PI / 2.0)) {
            this->last_yaw = 0;
        }
        vals[HEAD_YAW] = dyaw + last_yaw;
        double dpitch = asin(delta_y / dist);
        if (abs(dpitch + last_pitch) >= (PI / 6.0)) {
            this->last_pitch = 0;
        }
        vals[L_HEAD] = dpitch + last_pitch;

        return vals;
    }();
    move_group_->setJointValueTarget(target_joints);

    // Create a plan to the target Joint States
    auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan plmsg;
        auto const ok = static_cast<bool>(move_group_->plan(plmsg));
        return std::make_pair(ok, plmsg);
    }();

    // Move to target
    auto const ok = static_cast<bool>(move_group_->move());
    if (ok) {
        last_yaw = target_joints.at(HEAD_YAW);
        last_pitch = target_joints.at(L_HEAD);
        RCLCPP_INFO(this->get_logger(), "Successfully Executed!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Execution Failed!");
    }
}
