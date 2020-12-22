#include <nav2d_msgs/LocalizedScan.h>
#include <nav2d_msgs/LocalizedObject.h>
#include <ros/ros.h>

ros::Publisher FirstPubScan, SecondPubScan, ThirdPubScan;
ros::Publisher FirstPubObject, SecondPubObject, ThirdPubObject;

void LocalizedScanCallback(nav2d_msgs::LocalizedScan karto_scan){
    nav2d_msgs::LocalizedScan scan = karto_scan;
    FirstPubScan.publish(scan);
    SecondPubScan.publish(scan);
    ThirdPubScan.publish(scan);
}

void LocalizedObjectCallback(nav2d_msgs::LocalizedObject karto_object){
    nav2d_msgs::LocalizedObject object = karto_object;
    FirstPubObject.publish(object);
    SecondPubObject.publish(object);
    ThirdPubObject.publish(object);
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "ScanShare");
    ROS_INFO("ScanShareNode connected to roscore");
    ros::NodeHandle ScanShareNode("~");//ROS Handler - local namespace.

    std::string mInputTopic, mFirstOutputTopic, mSecondOutputTopic, mThirdOutputTopic;
    std::string mInputObjectTopic, mFirstOutputObjectTopic, mSecondOutputObjectTopic, mThirdOutputObjectTopic;

    //Parameters
    ScanShareNode.param("input_scan", mInputTopic, std::string("/karto_out"));
    ScanShareNode.param("output_first_scan", mFirstOutputTopic,std::string("/share_laser_scan_1"));
    ScanShareNode.param("output_second_scan", mSecondOutputTopic,std::string("/share_laser_scan_2"));
    ScanShareNode.param("output_third_scan", mThirdOutputTopic,std::string("/share_laser_scan_3"));
    ScanShareNode.param("input_object", mInputObjectTopic, std::string("/object_out"));
    ScanShareNode.param("output_first_object", mFirstOutputObjectTopic,std::string("/share_object_1"));
    ScanShareNode.param("output_second_object", mSecondOutputObjectTopic,std::string("/share_object_2"));
    ScanShareNode.param("output_third_object", mThirdOutputObjectTopic,std::string("/share_object_3"));
    //Subscribing
    ros::Subscriber scan_sub = ScanShareNode.subscribe(mInputTopic,100,&LocalizedScanCallback);
    ros::Subscriber object_sub = ScanShareNode.subscribe(mInputObjectTopic,100, &LocalizedObjectCallback);

    //Publishing
    FirstPubScan = ScanShareNode.advertise<nav2d_msgs::LocalizedScan>(mFirstOutputTopic,1);
    SecondPubScan = ScanShareNode.advertise<nav2d_msgs::LocalizedScan>(mSecondOutputTopic,1);
    ThirdPubScan = ScanShareNode.advertise<nav2d_msgs::LocalizedScan>(mThirdOutputTopic,1);
    FirstPubObject = ScanShareNode.advertise<nav2d_msgs::LocalizedObject>(mFirstOutputObjectTopic,1);
    SecondPubObject = ScanShareNode.advertise<nav2d_msgs::LocalizedObject>(mSecondOutputObjectTopic,1);
    ThirdPubObject = ScanShareNode.advertise<nav2d_msgs::LocalizedObject>(mThirdOutputObjectTopic,1);

    ros::Rate rate(40);
    while (ros::ok()) {

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
