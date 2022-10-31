#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;


int main(int argc, char** argv){
    ros::init(argc, argv, "height_estimator");

    ros::NodeHandle n;

    tf::TransformListener listener;
    ros::Publisher pub_tag_pose = n.advertise<geometry_msgs::PoseStamped>("tag_pose", 10);
    ros::Rate rate(10.0);
    while (n.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/landing", "/drone", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            // ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // cout << "x: " << transform.getOrigin().x() << endl;
        // cout << "y: " << transform.getOrigin().y() << endl;
        // cout << "z: " << transform.getOrigin().z() << endl;
        // cout << "-------------------" << endl;
        // tf::Quaternion q(
        // transform.getRotation().x(),
        // transform.getRotation().y(),
        // transform.getRotation().z(),
        // transform.getRotation().w());
        // tf::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        // cout << "r: " << roll * 180 / 3.14 << endl;
        // cout << "p: " << pitch * 180 / 3.14 << endl;
        // cout << "y: " << yaw * 180 / 3.14 << endl;

        geometry_msgs::PoseStamped pub_msg_tag;
        pub_msg_tag.header.stamp = ros::Time();
        pub_msg_tag.header.frame_id = "wamv";
        pub_msg_tag.pose.position.x = transform.getOrigin().x();
        pub_msg_tag.pose.position.y = transform.getOrigin().y();
        pub_msg_tag.pose.position.z = transform.getOrigin().z();
        pub_msg_tag.pose.orientation.x = transform.getRotation().x();
        pub_msg_tag.pose.orientation.y = transform.getRotation().y();
        pub_msg_tag.pose.orientation.z = transform.getRotation().z();
        pub_msg_tag.pose.orientation.w = transform.getRotation().w();

        pub_tag_pose.publish(pub_msg_tag);
        rate.sleep();
    }
    return 0;
};