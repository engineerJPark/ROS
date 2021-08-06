#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define PUB_NODE_NAME "cmd_vel" // name of node
#define SUB_NODE_NAME "cmd_vel_sub" // name of node
#define TOPIC_NAME "cmd_vel_topic" // name of topic : cmd_vel_pub

void messageCb(const geometry_msgs::Twist& cmd_vel){
    ROS_INFO("%f\n", cmd_vel.linear.x);
    ROS_INFO("%f\n", cmd_vel.linear.y);
    ROS_INFO("%f\n", cmd_vel.linear.z);
    ROS_INFO("%f\n", cmd_vel.angular.x);
    ROS_INFO("%f\n", cmd_vel.angular.y);
    ROS_INFO("%f\n", cmd_vel.angular.z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, SUB_NODE_NAME);
    ros::NodeHandle nh;

    // ros::Subscriber cmd_vel_subscriber(TOPIC_NAME, messageCb);
    ros::Subscriber cmd_vel_subscriber =nh.subscribe("cmd_vel", 10, messageCb);
    
    ros::spin();

    return 0;
}