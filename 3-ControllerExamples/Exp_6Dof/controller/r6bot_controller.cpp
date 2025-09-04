#include "ros2_control_demo_example/r6bot_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_demo_example
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
  // 初始化所有在 r6bot_controller.yaml 中定义的 ros__parameter_ 内容，存入 controller
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      //所有组合： "joint_1/position", "joint_1/velocity", "joint_2/position", ... "joint_6/velocity"
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      // "joint_1/position", "joint_1/velocity", "joint_2/position", ... "joint_6/velocity"
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  auto callback = [this](const trajectory_msgs::msg::JointTrajectory traj_msg) -> void
  {
    RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory.");
    //注意，在ros2 humble 版本中，下面这一行写作 traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    //humble 与 jazzy有差别的原因是：
    //从 Humble 到 Jazzy，ros2_control 把“实时-安全消息中转站”全部换成了新的 realtime_tools::RealtimeBuffer（简称 RTB）模板类，而不再使用旧的 realtime_tools::RealtimePublisher + writeFromNonRT() 机制。
    // Humble 时代
    // 控制器里通常有一个
    // realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>::Ptr traj_msg_external_point_ptr_;
    // 它内部自带一个线程安全的 FIFO，非实时线程用 writeFromNonRT(msg) 把消息塞进去，实时线程在 update() 里再用 readFromRT() 取出来。
    // 这套 API 名字里还带 “Publisher”，容易让人误以为它只是发话题，其实被拿来当“跨线程队列”用。
    // Jazzy 开始
    // ros2_control 统一改用 realtime_tools::RealtimeBuffer<T>。
    // 这是一个更轻量、语义更清晰的“实时缓冲区”——非实时侧 set()，实时侧 get()，内部只有一块互斥锁保护的双缓冲，拷贝一次就完事。
    // 于是原来的
    // traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    // 被直接替换成
    // traj_msg_external_.set(traj_msg);
    // （traj_msg_external_ 类型现在是 realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr>）。
    traj_msg_external_.set(traj_msg);
    new_msg_ = true;
  };

  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // 初始化成一个合理值
  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_1.positions[i];
  }
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.velocities[i] =
      delta * point_2.velocities[i] + (1.0 - delta) * point_1.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, bool & reached_end)
{
  double traj_len = static_cast<double>(traj_msg.points.size());
  auto last_time = traj_msg.points.back().time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;
  double cur_time_sec = cur_time.seconds();
  reached_end = (cur_time_sec >= total_time);

  // If we reached the end of the trajectory, set the velocities to zero.
  if (reached_end)
  {
    point_interp.positions = traj_msg.points.back().positions;
    std::fill(point_interp.velocities.begin(), point_interp.velocities.end(), 0.0);
    return;
  }

  size_t ind =
    static_cast<size_t>(cur_time_sec * (traj_len / total_time));  // Assumes evenly spaced points.
  ind = std::min(ind, static_cast<size_t>(traj_len) - 2);
  double delta = std::min(cur_time_sec - static_cast<double>(ind) * (total_time / traj_len), 1.0);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // 只有在 内置 new_msg_ 为 true 时，才代表需要更新一系列的 trajectory point
  if (new_msg_)
  {
    auto trajectory_msg_op = traj_msg_external_.try_get();
    if (trajectory_msg_op.has_value())
    {
      trajectory_msg_ = trajectory_msg_op.value();
      start_time_ = time;
      new_msg_ = false;
    }
  }

  if (!trajectory_msg_.points.empty())
  {
    bool reached_end;
    // 为了平滑移动，按时间等比例分，确定当前应当到达的位置
    interpolate_trajectory_point(trajectory_msg_, time - start_time_, point_interp_, reached_end);

    // If we have reached the end of the trajectory, reset it..
    if (reached_end)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory execution complete.");
      trajectory_msg_.points.clear();
    }

    for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    {
      if (!joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set position value for index %ld", i);
      }
    }
    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    {
      if (!joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set velocity value for index %ld", i);
      }
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

}  // namespace ros2_control_demo_example


// 为了导出 plugin，方便在 launch files 中好指定以供 controller manager 来 spawn
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example::RobotController, controller_interface::ControllerInterface)
