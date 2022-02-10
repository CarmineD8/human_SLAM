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
 
   
IMPORTANT:
For have much less perfomance issues with SLAM and be able to run at the speed of 1, is it important to build the catkin_ws with the following command:

```

catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

For some reason building it without this argument resulted in a much poorer perfomance of the system.



# Loop-closure 

OpemMapper.cpp contains the current algorithm of adding custom nodes and making links between them for the loop closure. For each custom node, physical, semantic distances as well as scan response are compared and those under the theshold are allowed to make a link between each other.

Custom nodes can be added in three ways:
  - voice commands from the user based on the onthologies. Voice commands are entered as indices which correspond to a word in the dictionary, which then can be translated to the corresponding
  - ros-keyboard module provides to enter the indices numbers with the keyboard
  - parse_bag.py in recognition repo automatically parses indices at the correct time. src/human_SLAM/navigation/nav2d_tutorials/launch/timed_bag.py is able to transform a normal bag file into a bag file with a topic which publishes current time.

Below you can see an example of uncorrected map
 ![rviz_screenshot_2021_03_22-16_17_27](https://user-images.githubusercontent.com/47984690/151000981-27b0c87c-28db-4c3c-83b8-962b0c2ec5ac.png)



And the same map, but with custom nodes added
![rviz_screenshot_2021_03_22-16_01_59](https://user-images.githubusercontent.com/47984690/151001082-22c1f839-e311-412e-a8bc-a2d47d5e2e4e.png)



# How to run

There are 4 launch files: 
   - streaming_to_holo.launch - streams live SLAM data from the sensors to the Microsoft Hololens 2. Compressed image from RVIZ is published as 
 topic to the ROS network which then can be picked up by the application in Unity running on Hololens 2 with the help of ROS_connector module. Before running this launch file, pedometer has to be launched from a seperate terminal with root access, as well as read_thingy91.py when proper connection with the cloud is established. Supports addition of custom nodes via voice input. Follow the folowing tutorial on how to setup the ROS bridge connection with Unity: https://www.youtube.com/watch?v=OZiAJuWh6w8&ab_channel=TheRealFran
 
   - from_vicoli_bag_only_voice.launch - plays a bag file recorded from vicoli and performs SLAM on it. Supports input with a voice as method of addition custom nodes. Two modes of operation: with and without ontology - more can be found in the launch file.
   - from_vicoli_bag_only_keyboard.launch - does the same as the previous, but with keyboard as a method of adding custom nodes. Mostly used for easier debigging. PLEASE NOTE: Switch to latest-keyboard branch to use this launch file.
   - from_vicoli_bag_only_automated.launch - works in tandem with exp_automated.bash script. Input: recorded timed (with current as a topic) + recorded responses from the experiment as a list of words. Output: 10 recorded trajectories of the operator + generated maps + logs. This data can later be used for further statistical prcoessing. Important: in the bash script don't forget to set Onto to true or false - depending on the data set. 
 




