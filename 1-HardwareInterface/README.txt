1.This is a tutorial for ROS2 controller hardware interface development.
2.For convenient, the start dir contains a fundamental robot description, the end dir contains the whole project with a finished hardware interface job.
3.To figure out how to develop a HW interface, you may simply compare two project dir(start & end).
4.Here are some counterpart points marked down to notice the difference between  two projects, which you may pay attention to:

    1)Dir my_robot_hardware contains the HW interface proj code: 
        include/my_robot_hardware/mobile_base_hardware_interface.hpp
        src/mobile_base_hardware_interface.cpp
        mobile_base.ros2_control.xacro (in dir: my_robot_description/urdf)
        my_robot_hardware_interface.xml
        CMakeLists.txt (insure each build requirements as been added in)
        add build requirements in package.xml
      these are files that necessary for an implement.
    2)Notice in mobile_base.ros2_control.xacro (in dir: my_robot_description/urdf):
        in <ros2_control <hardwar> tag: 
        use mock component at first :<plugin>mock_components/GenericSystem</plugin>
        once you have finished HW interface plugin (later in 3)), if you want to test real hardware you can change to tag: <plugin>mobile_base_hardware/MobileBaseHardwareInterface</plugin> (this is the plugin name set in my_robot_hardware_interface.xml) 
    3)For ROS2 invoke, the HW interface has to be built as a plugin:
        add lines in src/mobile_base_hardware_interface.cpp: PLUGINLIB_EXPORT_CLASS(<export class>, <base class for export class>)
        add lines in CmakeLists.txt: pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)
    4)If you want to test and run the project(the HW interface is done, below is just for test):
        make sure you have all the controllers used added in the my_robot_controllers.yml(in my_robot_bringup/config), together with all ros parameters well set.
        make sur the my_robot.launch.xml(in my_robot_bringup/launch) has been properly set.