# ROSControllerLearningCase
Contains a ROS2 project with tutorial 
Contains:
1. 1-HardwareInterface: hardware interface 开发样本工程 
2. 2-ROSController: ROS2 Controller 开发样本工程
3. 3-ControllerExamples: 举例学习、开发一些典型的 Controller
    1）6DoF: 6轴机械臂项目案例。详细的项目代码解读在该文件夹的README.txt 以及 该项目的file代码详细注解中。
    2）利用该项目，实现并测试了一个简单的基于大语言模型prompt生成ros2命令，然后发送到（具体情况是：我的本地大模型与机械臂ros2运行在不同的机器上，而发送顶层命令的机器位于外网的第三台机器上）目标机器上进行操控的案例。
    使用的 prompt 保存在项目文件夹中