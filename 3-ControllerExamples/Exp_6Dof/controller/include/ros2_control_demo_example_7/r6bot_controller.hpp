#ifndef ROS2_CONTROL_DEMO_EXAMPLE__R6BOT_CONTROLLER_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE__R6BOT_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace ros2_control_demo_example
{
class RobotController : public controller_interface::ControllerInterface
{
public:
  RobotController();

  // command_interface_configuration & state_interface_configuration 实现 controller 时需要定义，
  // 作用是将
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // update 函数与 hardware interface 中定义的 read, write 接口互动，执行顺序是 read -> update -> write 循环
  // update 函数可以灵活设置更新方式 
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // on_init 包含在 r6bot_controllers中定义的 ros__parameters_
  controller_interface::CallbackReturn on_init() override;

  // on_configure 函数对于lifecycle node必要,其中初始化subscription,并指定需要订阅的具体 topic
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // joint_names 需要与 usrd 中的 joint 名字 相同
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
  realtime_tools::RealtimeThreadSafeBox<trajectory_msgs::msg::JointTrajectory> traj_msg_external_;
  std::atomic<bool> new_msg_ = false;
  rclcpp::Time start_time_;
  trajectory_msgs::msg::JointTrajectory trajectory_msg_;
  trajectory_msgs::msg::JointTrajectoryPoint point_interp_;

  // 由于该controller 支持 两种模式 position 和 velocity ，因此 state interface 和 
  // command_interface 都要分别包含 position 和 velocity
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

  // interface 具体的名字举例: "joint_1/position", "joint_2/velocity"
  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_},
      {"velocity", &joint_velocity_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_}};
};

}  // namespace ros2_control_demo_example

#endif 
