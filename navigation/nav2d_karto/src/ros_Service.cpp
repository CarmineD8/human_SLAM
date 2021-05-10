// this header incorporates all the necessary #include files and defines the class "ros_Service"
#include "ros_Service.h"

using namespace std;
ros_Service::ros_Service()
{	
	//ros::init(argc, argv, "ros_Service");
	// Get parameters from the ROS parameter server
	ros::NodeHandle n;
	
	// n.param("SEMANTIC DISTANCE", distSubscriber, std::string("sem_dist"));
	// distSubscriber = n.subscribe(distTopic, 1, &ros_Service::getSemDist, this);
	indPublisher=n.advertise<std_msgs::Int32MultiArray>("Sem_indices",1,true);           
	

}


ros_Service::~ros_Service()
{
}

int ros_Service::getSemDist(std::int16_t obj1,std::int16_t obj2)
{
	std_msgs::Int32MultiArray array;
	array.data.clear();
	array.data.push_back(obj1);
	array.data.push_back(obj2);
	indPublisher.publish(array);
	

	return 1;
} 




    