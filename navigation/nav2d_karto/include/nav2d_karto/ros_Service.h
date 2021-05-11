// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef ros_Service_H_
#define ros_Service_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
// #include <image_msg/ImageLocation.h>
#include <OpenKarto/OpenKarto.h>
#include <list>

#include <string>
#include <map>




class ros_Service
{
public:
	// Constructor & Destructor
	ros_Service();
	~ros_Service();

	// Public methods
	

	int getSemDist(std::int16_t obj1,std::int16_t obj2);

	void callback_Dist(std_msgs::Int16 Dist);
private:
	// Private methods
	
	
	
	
	ros::Subscriber distSubscriber; 
	ros::Publisher indPublisher;

	std::string distTopic;
	std::int16_t Dist;



};
#endif