#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#define NODE_NAME "cmd_vel" // name of node
#define TOPIC_NAME "cmd_vel_topic" // name of topic : cmd_vel_pub

// need to group it as class

int main(int argc, char **argv){
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  geometry_msgs::Twist cmd_vel; // variable to publish

  //   0 to 6 in order...  linear.x;  linear.y;  linear.z;  angular.x;  angular.y;  angular.z;
  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_NAME, 100, true);

  ros::Rate loop_rate(10);

  std_msgs::Float64MultiArray msg;

  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
  msg.layout.dim[0].size = 6;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "twist";

  while (ros::ok()){
    msg.data.clear();

    msg.data.push_back(linear.x);
    msg.data.push_back(linear.y);
    msg.data.push_back(linear.z);
    msg.data.push_back(angular.x);
    msg.data.push_back(angular.y);
    msg.data.push_back(angular.z);

    ROS_INFO("Sending cmd_vel topic\n");
    cmd_vel_publisher.publish(msg); // Publishes 'msg' message
    ROS_INFO("Completed\n");

    loop_rate.sleep(); // Goes to sleep according to the loop rate defined above.
  }
  return 0;
}