#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using Float64MultiArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander{
public:
    
    // Constructor
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));

        joint_cmd_sub_ = node_->create_subscription<Float64MultiArray>(
            "joint_cmd", 10, std::bind(&Commander::JointCmdCallback, this, _1));

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_cmd", 10, std::bind(&Commander::PoseCmdCallback, this, _1));
    }

    // Arm
    void goToNamedPose(std::string pose_name)
    {   
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(pose_name);
        planAndExecute(arm_);
    }

    void goToJointPose(std::vector<double> &joint_values)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint_values);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        arm_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        if (!cartesian_path){
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            if (fraction == 1.0)
            {
                arm_->execute(trajectory);
            }
        }
        
    }

    // Gripper
    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }

private:
    void planAndExecute(std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success){
            interface->execute(plan);
        }
    }

    // subscriber callback
    void OpenGripperCallback(const Bool &msg){
        if (msg.data)
        {
            openGripper();
        }
        else
        {
            closeGripper();
        }
    }

    void JointCmdCallback(const Float64MultiArray::SharedPtr msg){
        auto joints = msg->data;
        if (joints.size()==6){
            goToJointPose(msg->data);
        }
    }

    void PoseCmdCallback(const PoseCmd &msg){
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    // Subscriber
    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
