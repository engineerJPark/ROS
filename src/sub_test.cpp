#include "ros/ros.h" 
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"

#define NODE_NAME "cmd_vel" // name of node
#define TOPIC_NAME "cmd_vel_topic" // name of topic : cmd_vel_pub



void messageCb( const std_msgs::Float64MultiArray& cmd_vel){
    for(int i = 0; i < 6; i++)
        ROS_INFO("%d\n", cmd_vel.data[i]);
}
  


int main(int argc, char **argv){
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;

    // ros::Subscriber cmd_vel_subscriber(TOPIC_NAME, messageCb);
    ros::Subscriber cmd_vel_subscriber =nh.subscribe("cmd_vel", 10, messageCb);
    
    ros::spin();

    return 0;
}