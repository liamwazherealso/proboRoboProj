#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "string.h"

#include <cmath>


using namespace std;



//Declaring this here to make it global in scope
ros::Publisher vel_pub;


geometry_msgs::Twist vel_msg;

//void callback(std_msgs::String door_scan_msg)
//{
//  vel_msg.linear.x = 4.0;
//  vel_pub.publish(vel_msg);

//}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "botControl");

  ros::NodeHandle n;

  
  ////Subscriber must get the door scan string
  //ros::Subscriber sub = n.subscribe("/robot/wall_door_sensor", 1000, callback);

  //Publisher must publish twist values for robot to use
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

  while (ros::ok())
  {

    ros::spinOnce();
    vel_msg.linear.x = 4.0;
    vel_pub.publish(vel_msg);
 

  }


  return 0;
}



