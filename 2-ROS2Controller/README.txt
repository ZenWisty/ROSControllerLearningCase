1.This is a tutorial for custom ROS2 controller development.
2.For convenient, the start dir contains a fundamental robot project, the end dir contains the whole project with a finished custom controller job.
3.To figure out how to custom a controller, you may simply compare two project dir(start & end).
4.Here are some counterpart points marked down to notice the difference between  two projects, which you may pay attention to:
    
    1)Dir my_robot_controllers contains the custom ROS2 controller proj code: 
        include/my_robot_controllers/my_controller.hpp
        src/my_controller.cpp
        arm.ros2_control.xacro (in dir: my_robot_description/urdf)
        my_controller_pugin.xml
        CMakeLists.txt (insure each build requirements as been added in)
        add build requirements in package.xml
      these are files that necessary for an custom controller implementation.
    2)
    3)For ROS2 invoke, the controller has to be built as a plugin:
        add lines in src/my_controller.cpp: PLUGINLIB_EXPORT_CLASS(<export class>, <base class for export class>)
        add lines in CmakeLists.txt: pluginlib_export_plugin_description_file(controller_interface my_controller_plugin.xml)
    4)If you want to test and run the project(the HW interface is done, below is just for test):
        make sure you have the custom controller added in the my_robot_controllers.yml(in my_robot_bringup/config), together with all ros parameters well set.
        make sure the my_robot.launch.xml(in my_robot_bringup/launch) add lines:
            <node pkg="controller_manager" exec="spawner" args="my_arm_controller" />
        and comment out lines:
            <!-- <node pkg="controller_manager" exec="spawner" args="arm_joints_controller" /> -->
        to avoid the hardware interface confict claimed.