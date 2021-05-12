// this header incorporates all the necessary #include files and defines the class "ros_Service"
#include "ros_Service.h"
#include <unistd.h>

using namespace std;
ros_Service::ros_Service()
{	
	//ros::init(argc, argv, "ros_Service");
	// Get parameters from the ROS parameter server
	ros::NodeHandle new_handle;
	//ros::CallbackQueue my_queue;
	client = new_handle.serviceClient<service_node::GetSemDist>("get_sem_dist");
	//ros::ServiceClient client=n.serviceClient<
	//n.param("sem_distance", distSubscriber, std::string("Sem_dist"));
	// distSubscriber = new_handle.subscribe("Sem_dist", 10, &ros_Service::callback_Dist, this);
	// indPublisher=new_handle.advertise<std_msgs::Int32MultiArray>("Sem_indices",1,true);           
	// Dist=0;
	
}


ros_Service::~ros_Service()
{
}

// void ros_Service::callback_Dist(std_msgs::Int16 DistR)
// {
// 	Dist=DistR.data;
// 	std::cout<<"ROS DIST IS "<<Dist<<std::endl;

// }



int ros_Service::getSemDist(std::int16_t obj1,std::int16_t obj2)
{
	srv.request.a=obj1;
	srv.request.b=obj2;
	if (client.call(srv))
	{
		return srv.response.sum;
	}

	
	
} 



    