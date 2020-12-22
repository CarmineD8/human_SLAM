#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher laser_pose_pub;
geometry_msgs::PoseStamped estimated_laser;

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_map_listener");
    ROS_INFO("laser_map_listener connected to roscore");
    ros::NodeHandle nh;

    std::string robot_name;

    if(!nh.getParam("tf_prefix", robot_name))
        ROS_ERROR("No robot is listening!");
    //Published to estimated position
    laser_pose_pub = nh.advertise<geometry_msgs::Pose>("/"+robot_name+"/estimated_laser", 50);

    tf::TransformListener mFrameListener;

    ros::Rate rate(125);
    while (ros::ok()){
        ros::spinOnce();
        tf::StampedTransform mLaserMapTransform;
        try{
            //Look at the transform between the human laser frame and the map frame
            //time(0) means "the latest available" transform in the buffer.
            mFrameListener.lookupTransform(robot_name+"/base_laser_link", robot_name+"/map", ros::Time(0), mLaserMapTransform);
        }

        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        estimated_laser.pose.position.x=mLaserMapTransform.getOrigin().x();
        estimated_laser.pose.position.y = mLaserMapTransform.getOrigin().y();
        estimated_laser.pose.orientation.z=mLaserMapTransform.getRotation().z();
        estimated_laser.pose.orientation.w=-mLaserMapTransform.getRotation().w();
        estimated_laser.header.stamp = ros::Time::now();
        laser_pose_pub.publish(estimated_laser);
        rate.sleep();
    }
    return 0;
};
