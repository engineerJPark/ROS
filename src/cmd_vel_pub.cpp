#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define PUB_NODE_NAME "cmd_vel" // name of node
#define SUB_NODE_NAME "cmd_vel_sub" // name of node
#define TOPIC_NAME "cmd_vel_topic" // name of topic : cmd_vel_pub

int main(int argc, char **argv){
  ros::init(argc, argv, PUB_NODE_NAME);
  ros::NodeHandle nh;
  geometry_msgs::Twist cmd_vel; // variable to publish

  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_NAME, 100, true);
  ros::Rate loop_rate(10);

  while (ros::ok()){
    ROS_INFO("Sending cmd_vel topic\n");
    cmd_vel_publisher.publish(cmd_vel); // Publish Twist cmd_vel
    ROS_INFO("Completed\n");

    loop_rate.sleep(); // Goes to sleep according to the loop rate defined above.
  }

  return 0;
}