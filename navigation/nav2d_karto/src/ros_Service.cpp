// this header incorporates all the necessary #include files and defines the class "ros_Service"
#include "ros_Service.h"

using namespace std;
ros_Service::ros_Service()
{	
	//ros::init(argc, argv, "ros_Service");
	// Get parameters from the ROS parameter server
	ros::NodeHandle n;
	
	//n.param("sem_distance", distSubscriber, std::string("Sem_dist"));
	distSubscriber = n.subscribe("Sem_dist", 10, &ros_Service::callback_Dist, this);
	indPublisher=n.advertise<std_msgs::Int32MultiArray>("Sem_indices",1,true);           
	

}


ros_Service::~ros_Service()
{
}

void ros_Service::callback_Dist(std_msgs::Int16 DistR)
{
	Dist=DistR.data;
	std::cout<<"ROS DIST IS "<<Dist<<std::endl;

}

int ros_Service::getSemDist(std::int16_t obj1,std::int16_t obj2)
{
	std_msgs::Int32MultiArray array;
	
	array.data.push_back(obj1);
	array.data.push_back(obj2);
	indPublisher.publish(array);
	ros::spinOnce();

	return Dist;
	
} 



    