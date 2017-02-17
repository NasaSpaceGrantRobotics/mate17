/*
 * Prints the recievied signal read by the bluetooth module
 * Uses ROS to publish the read signal, which can be accessed via 
 * terminal
 *
 *
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher bluetooth("bluetooth", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(bluetooth);
}

void loop()
{
  str_msg.data = hello;
  bluetooth.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
