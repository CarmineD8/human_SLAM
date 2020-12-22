#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.141592658
//ROS msgs
geometry_msgs::Twist body, head;
geometry_msgs::Twist odom_original, odom_previous, odom_correct, odom_correct_previous;
bool HeadPoseReceived = false;
ros::Publisher scan_pub;

struct Difference{
    float x;
    float y;
    float z;
};

void HeadPoseCallback(const geometry_msgs::Twist::ConstPtr& head_pose){
    head.linear.x = head_pose->linear.x;
    head.linear.y = head_pose->linear.y;
    head.angular.z = head_pose->angular.z;
    HeadPoseReceived = true;
}

void BodyPoseCallback(const geometry_msgs::Twist::ConstPtr& body_pose){
    body.linear.x = body_pose->linear.x;
    body.linear.y = body_pose->linear.y;
    body.angular.z = body_pose->angular.z;
}

void LaserScanCallback(const sensor_msgs::LaserScan scan){
    sensor_msgs::LaserScan laserScan;
    laserScan = scan;
    laserScan.header.stamp = ros::Time::now();
    scan_pub.publish(laserScan);
}


int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "odom_laser_transform");
    ROS_INFO("odom_laser_transform connected to roscore");
    ros::NodeHandle nh;//ROS Handler - local namespace.

    std::string robot_name;

    if(!nh.getParam("tf_prefix", robot_name))
        ROS_ERROR("No detecting robots!");

    //Subscribing
    ros::Subscriber body_sub = nh.subscribe("/"+robot_name+"/body_pose", 50, &BodyPoseCallback);
    ros::Subscriber scan_sub = nh.subscribe("/"+robot_name+"/base_scan", 50, &LaserScanCallback);
    ros::Subscriber head_sub = nh.subscribe("/"+robot_name+"/head_pose", 50, &HeadPoseCallback);

    //Publishing
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/"+robot_name+"/base_scan2",50);

    //Initializations
    Difference diff, diff_previous;
    diff.x = 0;
    diff.y = 0;
    diff.z = 0;
    diff_previous.x = 0;
    diff_previous.y = 0;
    diff_previous.z = 0;
    odom_previous.linear.x = 0;
    odom_previous.linear.y = 0;
    odom_previous.angular.z = 0;

    ros::Rate rate(3);
    while (ros::ok()) {
        ros::spinOnce();

        odom_original.linear.x = body.linear.x;
        odom_original.linear.y = body.linear.y;
        odom_original.angular.z = head.angular.z;

        diff.x = odom_original.linear.x - odom_previous.linear.x;
        diff.y = odom_original.linear.y - odom_previous.linear.y;
        diff.z = odom_original.angular.z - odom_previous.angular.z;

        //The range of the heading is within the range [-pi,pi]
        if(diff.z > PI){
            diff.z -= 2*PI;
        }
        if(diff.z < -PI){
            diff.z += 2*PI;
        }

        if(HeadPoseReceived){
            if(fabs(diff.z)>1){
                diff.z /= 300;
                odom_correct.angular.z = odom_previous.angular.z + diff.z;
            } else if(fabs(diff.z)>0.2){
                diff.z /= 30;
                odom_correct.angular.z = odom_previous.angular.z + diff.z;
            }

            if(fabs(diff.x)>0.6){
                odom_correct.linear.x = odom_correct_previous.linear.x;
            }else{
                odom_correct.linear.x = odom_correct_previous.linear.x + diff.x;
                diff_previous.x = diff.x;
            }

            if(fabs(diff.y)>0.6){
                odom_correct.linear.y = odom_correct_previous.linear.y;
            }else{
                odom_correct.linear.y = odom_correct_previous.linear.y + diff.y;
                diff_previous.y = diff.y;
            }
        }else{
            int correction = -1 + rand()%3;
            diff.x = diff_previous.x;
            diff.y = diff_previous.y;
            diff.z = diff_previous.z;

            odom_correct.linear.x = odom_correct_previous.linear.x + diff.x/10.0;
            odom_correct.linear.y = odom_correct_previous.linear.y + diff.y/10.0;
            odom_correct.angular.z = odom_original.angular.z + float(correction)/20.0;
        }

        static tf::TransformBroadcaster mFramebroadcaster;
        tf::Transform mLaserBaseTransform, mOdomLaserTransform;
        tf::Quaternion mLaserBaseQuaternion, mOdomLaserQuaternion;

        mLaserBaseTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        mLaserBaseQuaternion.setRPY(0.0, 0.0, body.angular.z-head.angular.z);
        mLaserBaseTransform.setRotation(mLaserBaseQuaternion);

        mOdomLaserTransform.setOrigin(tf::Vector3(head.linear.x, head.linear.y, 0.0));
        mOdomLaserQuaternion.setRPY(0.0, 0.0, head.angular.z);
        mOdomLaserTransform.setRotation(mOdomLaserQuaternion);

        mFramebroadcaster.sendTransform(tf::StampedTransform(mLaserBaseTransform, ros::Time::now(), robot_name+"/laser", robot_name+"/base_link"));
        mFramebroadcaster.sendTransform(tf::StampedTransform(mOdomLaserTransform, ros::Time::now(), robot_name+"/odom", robot_name+"/laser"));

        odom_previous = odom_original;
        odom_correct_previous = odom_correct;
        diff_previous.z = diff.z;

        HeadPoseReceived = false;

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
