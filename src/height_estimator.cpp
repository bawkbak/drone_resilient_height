#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std; 

class Height{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_height;
        ros::Subscriber sub_uwb_position;
        ros::Subscriber sub_tag_position;

        geometry_msgs::PoseStamped msg_tag;
        geometry_msgs::PoseStamped msg_uwb;
        float height = 0;
    public:
        Height();
        void uwbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void tagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void estimator();
};

Height :: Height(){
    pub_height = n.advertise<geometry_msgs::PoseStamped>("height_estimator/relative_pose", 10);
    sub_uwb_position = n.subscribe<geometry_msgs::PoseStamped>("uwb_pose", 1,  &Height::uwbCallback, this);
    sub_tag_position = n.subscribe<geometry_msgs::PoseStamped>("tag_pose", 1,  &Height::tagCallback, this);
}

void Height :: uwbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_uwb = *msg;
    return;
}

void Height :: tagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_tag = *msg;
    return;
}

void Height :: estimator(){
    ros::Rate r(10);
    // double current_time = ros::Time::now().toSec();
    // double uwb_time = ros::Time::now().toSec();
    // double tag_time = ros::Time::now().toSec();
    cout << "--------------------------" << endl;
    cout << "TAG X: " << msg_tag.pose.position.x << endl;
    cout << "TAG Y: " << msg_tag.pose.position.y << endl;
    cout << "TAG Z: " << msg_tag.pose.position.z << endl;
    cout << endl << endl;
    cout << "UWB X: " << msg_uwb.pose.position.x << endl;
    cout << "UWB Y: " << msg_uwb.pose.position.y << endl;
    cout << "UWB Z: " << msg_uwb.pose.position.z << endl;
    r.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "height_estimator");
    Height drone;
    while(ros::ok()){
        drone.estimator();
        ros::spinOnce();
    }
    return 0;
}