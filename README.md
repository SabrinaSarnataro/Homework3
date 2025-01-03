# Homework3

WARNING: Inside the Docker file provided by the professor, the world URI is associated with the following path:
RUN echo "export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models" >> ${HOME}/.bashrc

Please, once you have cloned the repository, open the Homework3 folder, copy the files into the directory associated with the Docker container and delete the old one.

After cloning the repository, build the packages by doing:

     $ colcon build

Then, use the source command:

    $ source install/setup.bash

To test the point 1, first of all you have to change the world in the launch file. To do this go into ros2_iiwa/iiwa_bringup/launch and change "empty.world" with "spherical.world" at line 326 in the iiwa.launch.py.

Then you can write in a terminal:

     $ ros2 launch iiwa_bringup iiwa.launch.py use_vision:=true

In another terminal write:

     $ros2 run ros2_opencv ros2_opencv_node

To check the image seen by the camera you can write:

     $ ros2 run rqt_image_view rqt_image_view

Select the topic /videocamera to see the real image or /videocamera_processata if you prefer to see the detected image.

If you want to spawn the robot by using the velocity controller (as requested in the point 2.a), write the following command in a terminal:

    $ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_vision:=true

In a second terminal run the "single" node contained in the package "aruco_ros" with the following command if you want the robot to exhibit a positioning behavior (as requested in the point 2.a.i):

    $ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=world -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical 

Run the following command instead if you want the robot to exhibit a look-at-point behavior (as requested in the point 2.a.ii):

    $ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical 


In a third terminal run the "ros2_kdl_vision_control" contained in the package "ros2_kdl_package" as follows if you want the positioning behaviour:

    $ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p point_2a:=1

If you want a look-at-point behaviour instead you have to write:

    $ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p point_2a:=2


If you want to spawn the robot by using the effort controller (as requested in the point 2.b), write in the first terminal the following command:

    $ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_vision:=true

In the second terminal you have to write the following command:

    $ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical 


In the third terminal, you can run the node as follows if you want to use the control law with the joints space inverse dynamics controller:

    $ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort -p choice_dyn:=1

If you want to use the control law with the Cartesian space inverse dynamics controller instead, you have to write the following command:

    $ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort -p choice_dyn:=2


If you want to monitor the image captured by the robot's camera, you can use the following command:

    $ ros2 run rqt_image_view rqt_image_view

To see the real image, select the /videocamera topic, to see the image with the detection of the ArUco tag instead, select the aruco_single/result topic.

You can check the behaviour of the robot by visiting the following link 
     https://www.youtube.com/watch?v=grszz5pnCko
     
