#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/State.h>

using namespace std; 

class Height{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_base_position;
        ros::Publisher pub_height;


        ros::Subscriber sub_wamv_position;
        ros::Subscriber sub_uwb_position;
        ros::Subscriber sub_tag_position;
        ros::Subscriber sub_drone_position;
        
        ros::Subscriber sub_drone_state;

        geometry_msgs::PoseStamped msg_pose_tag;
        geometry_msgs::PoseStamped msg_pose_uwb;
        geometry_msgs::PoseStamped msg_pose_wamv;

        float height_estimated = 0;
        float height_offset = 0;
        bool drone_is_armed = false;
    public:
        Height();
        void uwbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void tagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void wamvPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void droneStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void estimator();
};

Height :: Height(){
    pub_base_position = n.advertise<geometry_msgs::PoseStamped>("height_estimator/relative_pose", 10);
    pub_height = n.advertise<std_msgs::Float32>("height_estimator/relative_height", 10);

    sub_wamv_position = n.subscribe<geometry_msgs::PoseStamped>("wamv_pose", 1,  &Height::wamvPoseCallback, this);
    sub_uwb_position = n.subscribe<geometry_msgs::PoseStamped>("uwb_pose", 1,  &Height::uwbCallback, this);
    sub_tag_position = n.subscribe<geometry_msgs::PoseStamped>("tag_pose", 1,  &Height::tagCallback, this);
    sub_drone_position = n.subscribe<geometry_msgs::PoseStamped>("drone_pose", 1,  &Height::dronePoseCallback, this);
    sub_drone_state = n.subscribe<mavros_msgs::State>("drone_state", 1,  &Height::droneStateCallback, this);
}



void Height :: uwbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_pose_uwb = *msg;
    return;
}

void Height :: tagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_pose_tag = *msg;
    return;
}

void Height :: wamvPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_pose_wamv = *msg;
    return;
}

void Height :: dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(drone_is_armed){
        height_estimated = msg->pose.position.z - height_offset;
        std_msgs::Float32 msg_height;
        msg_height.data = height_offset;
        pub_height.publish(msg_height);
        cout << "Height: " << height_estimated << endl;
    }else{
        cout << "LAND" << endl;
        height_offset = msg->pose.position.z;
    }
    return;
}

void Height :: droneStateCallback(const mavros_msgs::State::ConstPtr& msg){
    drone_is_armed = msg->armed;
    return;
}

void Height :: estimator(){
    ros::Rate r(10);
    // double current_time = ros::Time::now().toSec();
    // double uwb_time = ros::Time::now().toSec();
    // double tag_time = ros::Time::now().toSec();

    double tmp_r, tmp_p, tmp_y;
    tf::Quaternion q(msg_tag.pose.orientation.x, msg_tag.pose.orientation.y, msg_tag.pose.orientation.z, msg_tag.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(tmp_r, tmp_p, tmp_y);
    cout << "--------------------------" << endl;
    cout << "TAG X: " << msg_pose_tag.pose.position.x << endl;
    cout << "TAG Y: " << msg_pose_tag.pose.position.y << endl;
    cout << "TAG Z: " << msg_pose_tag.pose.position.z << endl;
    cout << "TAG YAW: " << tmp_y * 180 /3.1415926 << endl;
    cout << endl << endl;
    cout << "UWB X: " << msg_pose_uwb.pose.position.x << endl;
    cout << "UWB Y: " << msg_pose_uwb.pose.position.y << endl;
    cout << "UWB Z: " << msg_pose_uwb.pose.position.z << endl;
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