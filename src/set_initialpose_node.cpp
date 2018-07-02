#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char** argv) 
{

    // ROS
    ros::init(argc, argv, "set_initialpose");
    ros::NodeHandle node_handle;

    // Publisher for initial pose
    ros::Publisher pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);

    ros::Duration(5.0).sleep(); // sleep for 5 sec 


    // get parameter from launch file
    double position_x, position_y, orientation_th;
    ros::param::get("~position_x", position_x);
    ros::param::get("~position_y", position_y);
    ros::param::get("~orientation_th", orientation_th);

    double covariance_x, covariance_y, covariance_th;
    ros::param::get("~covariance_x", covariance_x);
    ros::param::get("~covariance_y", covariance_y);
    ros::param::get("~covariance_th", covariance_th);

    // init pose with covariance
    geometry_msgs::PoseWithCovarianceStamped msg;

    // set time
    msg.header.stamp = ros::Time::now();;

    // set tf frame
    msg.header.frame_id = "map";

    // set 2D position
    msg.pose.pose.position.x = position_x;
    msg.pose.pose.position.y = position_y;
    msg.pose.pose.position.z = 0;

    // set 2D orientation
    tf::Quaternion q;
    q.setRPY(0, 0, orientation_th);
    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.orientation.w = q[3];
    msg.pose.covariance.assign(0);

    // set covariances
    msg.pose.covariance[6 * 0 + 0] = covariance_x; // position x
    msg.pose.covariance[6 * 1 + 1] = covariance_y; // position y
    msg.pose.covariance[6 * 5 + 5] = covariance_th; // orientation

   ros::Rate loop_rate(0.0005);


   int count = 0;

  while (ros::ok())
  {
    if(count == 0)
    {
        pub.publish(msg);
        ROS_INFO("set initial pose to x = %f y = %f yaw = %f", position_x,position_y,orientation_th);
    }
    loop_rate.sleep();
    count += 1;
  }
    return 0;

}
