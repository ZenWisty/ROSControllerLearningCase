## 4-ROS2ArmMoveit: 建立 Robotic Arm 并使用 Moveit Cpp API

### Project Overview

1. my_robot_description: 用于描述机械臂的urdf 模型, 包括 controller.xacro 关键信息
2. my_robot_moveit_config: 由 moveit GUI 工具包生成的内一个工程，此处已经在构建完 my_robot_description 工程后配置好，可直接搬运使用，无须编辑。
3. my_robot_interface: 包含需要用到的一个自定义interface
4. my_robot_commander_cpp: 包含了使用 moveit cpp api 开发的若干的交互接口和subscriber，用于控制机械臂：<br>
    Commander::OpenGripperCallback<br>
    Commander::JointCmdCallback<br>
    Commander::PoseCmdCallback<br>
5. my_robot_bringup: 用于启动机械臂的bringup 包

### Demo 使用方法
在安装了 ROS jazzy 和 moveit 的环境中。

### Details
1. 如果您要从头自己参考教程配置 moveit 工程（不确定是否只有我遇见这个问题），建议在配置完工程后，检查 moveit_controllers.yml 中如 type: FollowJointTrajectory 类的 controller 配置，确保其中包含了 action_ns: follow_joint_trajectory 关键字信息。<br>
否则可能导致moveit 在启动时，无法找到对应的 controller 启动信息，使得 configure过程失败，后续会有连串错误无法控制相应控件，该错误的提示信息为：
```
[move_group-3] [ERROR] [1758807803.282355251] [move_group.moveit.moveit.plugins.simple_controller_manager]: No action namespace specified for controller `gripper_controller` through parameter `moveit_simple_controller_manager.gripper_controller.action_ns`
```
