#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "ardrone_brown/Navdata.h"

#include <time.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

using namespace std;

geometry_msgs::Twist twist;
int altd = -1;
int countDoku = 0;
void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
	altd = navdata->altd;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff");
  ros::NodeHandle node_handle;
  ros::Publisher pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, navdataUpdate);


  time_t start = time(NULL);
  ros::Rate loop_rate(1);
  bool dir = true;
  while(ros::ok())
  {
    twist.linear.y = dir ? 0.05 : -0.05;
    dir = !dir;
  ostringstream ostr;
  ostr << "linear.x: " << twist.linear.x << endl;
  ostr << "linear.y: " << twist.linear.y << endl << endl;
  ostr << "linear.z " << twist.linear.z << endl;
  ostr << "altd: " << altd << endl;
  ostr << "dir: " << dir << endl;
  ostr << "count: " << countDoku++ << endl;
  ROS_INFO(ostr.str().c_str());
    pub.publish(twist);
    loop_rate.sleep();
    /*ros::spinOnce();
    if(altd < 800)
      twist.linear.z = 0.4;
    else 
      twist.linear.z = 0;
    if(altd > 1000)
      twist.linear.z = -0.2;
    if(twist.linear.z == 0)
    {
    if(dir)
    {
      //twist.linear.x = 0.05;
    }
    //else
      //twist.linear.x = -0.05;
    }
    else
      twist.linear.x = 0;
    if(time(NULL) - start >= 1)
    {
      dir = !dir;
      start = time(NULL);
    }
  ostringstream ostr;
  ostr << "linear.x: " << twist.linear.x << endl;
  ostr << "linear.y: " << twist.linear.y << endl << endl;
  ostr << "linear.z " << twist.linear.z << endl;
  ostr << "altd: " << altd << endl;
  ostr << "dir: " << dir << endl;
  ROS_INFO(ostr.str().c_str());
    pub.publish(twist);
    loop_rate.sleep();*/
  }
  
}
