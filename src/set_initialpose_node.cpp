#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "iostream"
#include "ipa_navigation_msgs/StateEKF.h"
#include "ros/ros.h"
#include <ros/console.h>
#include "ros/package.h"
#include "std_msgs/String.h"
#include "string"
#include <sstream>
#include <tf_conversions/tf_eigen.h>

void safePose(double x, double y, double th);

void callbackPose(const ipa_navigation_msgs::StateEKF::ConstPtr& msg)
{
    // get robot pose from EKF. This Pose referres to map coordinate system
    double position_x = msg->state.pose.pose.position.x; // position x
    double position_y = msg->state.pose.pose.position.y; // position x

    tf::Quaternion q(                             // orientation comes as Quaternion -> Convert to euler to get theta
        msg->state.pose.pose.orientation.x,
        msg->state.pose.pose.orientation.y,
        msg->state.pose.pose.orientation.z,
        msg->state.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, orientation_th; // just dummy because we focus on one orientation in 2D
    m.getRPY(roll, pitch, orientation_th);

    static int counter;
    if(++counter == 300) // each second
    {
        safePose(position_x, position_y, orientation_th); // safe parameter in file
        counter = 0;
    }
}

void safePose(double x, double y, double th)
{
    std::string file = ros::package::getPath("set_initialpose");
    file.append("/cfg/pose.yaml");
    std::ofstream pose_file (file.c_str());

    if (pose_file.is_open())
    {
        pose_file <<" Position x = \n";
        pose_file << x;
        pose_file <<"\n Position y = \n";
        pose_file << y;
        pose_file <<"\n Orientation th = \n";
        pose_file << th;
        pose_file.close();
        //ROS_INFO("saved pose %f %f %f", x, y, th);
    }
    else std::cout << "Unable to open file";
}


geometry_msgs::PoseWithCovarianceStamped getInitPoseFromLaunchFile()
{
    double position_x, position_y, orientation_th;
    ros::param::get("~position_x", position_x);
    ros::param::get("~position_y", position_y);
    ros::param::get("~orientation_th", orientation_th);


    // init pose with covariance
    geometry_msgs::PoseWithCovarianceStamped msg;

    // set time
    msg.header.stamp = ros::Time::now();;

    // set tf frame
    msg.header.frame_id = "/map";

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

    // get covariances from param server
    double covariance_x, covariance_y, covariance_th;
    ros::param::get("~covariance_x", covariance_x);
    ros::param::get("~covariance_y", covariance_y);
    ros::param::get("~covariance_th", covariance_th);


    // set covariances
    msg.pose.covariance[6 * 0 + 0] = covariance_x; // position x
    msg.pose.covariance[6 * 1 + 1] = covariance_y; // position y
    msg.pose.covariance[6 * 5 + 5] = covariance_th; // orientation

    ROS_INFO("Send init pose to params in launch file %f %f %f ", position_x, position_y, orientation_th );
    return msg;

}


geometry_msgs::PoseWithCovarianceStamped getInitPoseFromFile()
{
    double position_x, position_y, orientation_th;

    position_x = 0.0;
    position_y = 0.0;
    orientation_th = 0.0;

    std::string file = ros::package::getPath("set_initialpose");
    file.append("/cfg/pose.yaml");

    std::ifstream f;  // file handle
    std::string name;
    f.open(file.c_str(), std::ios::in); // open file
    int count = 0;

    std::string::size_type sz;

    while (!f.eof())   // as long as we ve data
    {

        std::getline(f, name);        // read line
        if (count == 1)
        {
            position_x = std::stod(name, &sz);
        }
        if (count == 3)
        {
            position_y = std::stod(name, &sz);
        }
        if (count == 5)
        {
            orientation_th = std::stod(name, &sz);
        }
        count += 1;
    }
    f.close(); // Datei wieder schließen

    // init pose with covariance
    geometry_msgs::PoseWithCovarianceStamped msg;

    // set time
    msg.header.stamp = ros::Time::now();;

    // set tf frame
    msg.header.frame_id = "/map";

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

    // get covariances from param server
    double covariance_x, covariance_y, covariance_th;
    ros::param::get("~covariance_x", covariance_x);
    ros::param::get("~covariance_y", covariance_y);
    ros::param::get("~covariance_th", covariance_th);

    // set covariances
    msg.pose.covariance[6 * 0 + 0] = covariance_x; // position x
    msg.pose.covariance[6 * 1 + 1] = covariance_y; // position y
    msg.pose.covariance[6 * 5 + 5] = covariance_th; // orientation

    ROS_INFO("Send init pose to last known Pose: %f %f %f ", position_x, position_y, orientation_th );
    return msg;
}

int main(int argc, char** argv) 
{

    // ROS
    ros::init(argc, argv, "set_initialpose");
    ros::NodeHandle node_handle;

    // Publisher for initial pose
    ros::Publisher pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
    ros::Subscriber sub_pose_ekf = node_handle.subscribe<ipa_navigation_msgs::StateEKF>("/state_ekf", 10, callbackPose);

    ros::Duration(5.0).sleep(); // wait until localization software is launched

    bool use_last_known_pose;
    ros::param::get("~use_last_known_pose", use_last_known_pose);

    // send initial pose

    geometry_msgs::PoseWithCovarianceStamped init_pose;

    if(use_last_known_pose ==true)
    {
        init_pose = getInitPoseFromFile();
    }
    else
    {
        init_pose = getInitPoseFromLaunchFile();
    }
    pub.publish(init_pose);

    ros::spin();
    return 0;
}
