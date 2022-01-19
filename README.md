# Packages for human localization and mapping

The folder contains three packages:
  - navigation_2d
  - odom_laser_tf
  - sharelaser
  
Navigation_2d is the indigo version of the https://github.com/skasperski/navigation_2d package. With respect to this standard version, the mapping algorithm has been modified by adding the possibility of receiving the position of some object in the map (so as to close the loop by using an image matching procedure in parallel to the default loop clouser approach. See function sendLocalizedObject and receiveLocalizedObject in the MultiMapper.cpp).

Moreover, acquisitions have been performed in the historical city centre of Genoa, and collected in different bag files (folder bag_files in nav2d_tutorials/launch). These bag files (which contain the estimated human odometry (head and body frame) and the laser scanner acquisition) may be used for building a map of the enviroment. This can be done with one human operator (zena_one_robot.launch in nav2d_tutorials/launch) or with multiple operators. 

Please notice that when building the map, the rosbag file is usually played with a reduced speed (0.1), or the map will not be properly built.

Example:

```
roslaunch nav2d_tutorials zena_two_robots.launch
```

The other two packages are used for building the frame related to the lasers for the different human operators, and to publish laser data in different topics, which is necessary for building the map in a collaborative way.

There are three two launch files: 
   - streaming_to_holo.launch - streams live data SLAM from the sensors to the Microsoft Hololens 2. Compressed image from RVIZ is published as 
 topic to the ROS network which then can be picked up by the application in Unity running on Hololens 2 with the help of ROS_connector module.
   - from_vicoli_bag.launch - plays a bag file recorded from vicoli and performs SLAM on it.
   
   
IMPORTANT:
For have much less perfomance issues with SLAM and be able to run at the speed of 1, is it important to build the catkin_ws with the following command:

```

catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

For some reason building it without this argument resulted in a much poorer perfomance of the system.
