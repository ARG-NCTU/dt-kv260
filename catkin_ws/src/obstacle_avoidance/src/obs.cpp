#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Twist.h>
#include <stdio.h>

float min_range,min_range_angle;
bool flag;
ros::Publisher vel_pub;
geometry_msgs::Twist cmdvel;
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // if (!flag) return;
 // printf("Position: [%f] [%f]\n", msg->range_min,msg->range_max);
	min_range=msg->ranges[0];
	min_range_angle=0;
	for(int j=0;j<=241;j++) //increment by one degree
		{
		  	if(msg->ranges[j]<min_range && msg->ranges[j]!=0)
			{
				min_range=msg->ranges[j];
				min_range_angle=j/2;
      }
		}
		printf("minimum range is [%f] at an angle of [%f]\n",min_range,min_range_angle);
	if(min_range<=0.5)  // min_range<=0.5 gave box pushing like behaviour, min_range<=1.2 gave obstacle avoidance
	{
		if(min_range_angle<90)
		{
			 cmdvel.angular.z=0.25;
			 cmdvel.linear.x=0;
			 printf("left\n");
		}
		else
		{
			 cmdvel.angular.z=-0.25;
			 cmdvel.linear.x=0;
			 printf("right\n");
		}
	}
	else
	{
		cmdvel.linear.x=0.4;
		cmdvel.angular.z=0;
		printf("straight\n");
	}

	 if(flag) vel_pub.publish(cmdvel);

}
void joystickHandler(const sensor_msgs::Joy::ConstPtr& msg)
{
  if(msg->buttons[8]==1){
    printf("--------------------switch to manual\n");
    flag = 0;
    cmdvel.linear.x=0;
    cmdvel.linear.y=0;
    cmdvel.linear.z=0;
    cmdvel.angular.x=0;
    cmdvel.angular.y=0;
    cmdvel.angular.z=0;
    vel_pub.publish(cmdvel);
  }
  if(msg->buttons[9]==1){
    printf("--------------------switch to auto\n");
    flag = 1;
  }
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "laser_messages");
  ros::NodeHandle nh;
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  cmdvel.linear.x=0;
  cmdvel.linear.y=0;
  cmdvel.linear.z=0;
  cmdvel.angular.x=0;
  cmdvel.angular.y=0;
  cmdvel.angular.z=0;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan> ("scan", 1, chatterCallback);
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("joy", 1, joystickHandler);

  ros::spin();

  return 0;
}
