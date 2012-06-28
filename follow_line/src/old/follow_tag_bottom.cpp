#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include "ardrone_brown/Navdata.h"
#include "ar_recog/Tags.h"
#include "ar_recog/Tag.h"
#include "ardrone_brown/Navdata.h"

#include "Delta.h"

#include <time.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

using namespace std;

geometry_msgs::Twist twist;


ros::Publisher pub;

int altd = -1;

bool vor = false;
bool zurueck = false;
bool links = false;
bool rechts = false;

time_t sinceNotSeen;

bool seen = false;
bool dir = false;

Delta lxDelta;
Delta lyDelta;
Delta lzDelta;
Delta azDelta;


void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
	altd = navdata->altd;
}

void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
	float zr = 0;
  if(msg->tag_count == 0)
  {
    if(seen)
      sinceNotSeen = time(NULL);
    if(time(NULL) - sinceNotSeen < 1)
    {
      if(vor)
        twist.linear.x = lxDelta.get_velocity(0.05);
      if(zurueck)
        twist.linear.x = lxDelta.get_velocity(-0.05);
      if(links)
        twist.linear.y = lyDelta.get_velocity(0.05);
      if(rechts)
        twist.linear.y = lyDelta.get_velocity(-0.05);
    }
    else //Versuche ruhig in Luft zu stehen
    {
        twist.linear.x = 0;
        twist.linear.z = 0;
        //twist.linear.y = dir ? 0.2 : -0.2;
        twist.linear.y = 0;
        dir = !dir;
        twist.angular.z = 0;
    }
    seen = false;
  }
  else
  {
	  seen = true;
	 // int width = msg->image_width;
	 // int height = msg->image_height;
	  int width = 160;
	  int height = 120;

	  ar_recog::Tag biggest = msg->tags[0];

	  for(int i = 0; i < msg->tag_count; ++i)
	  {
	    if(msg->tags[i].diameter > biggest.diameter)
	      biggest = msg->tags[i];
	  }

	  float cx = 0;
	  float cy = 0;
	  for(int i = 0; i < 7; i+=2)
	  {
	    cx = cx + biggest.cwCorners[i];
	    cy = cy + biggest.cwCorners[i+1];
	  }
	  cx = cx / 4.0 / width;
	  cy = cy / 4.0 / height;



	  if(biggest.distance > 850.0f)
	  {
	    twist.linear.z = lzDelta.get_velocity(-0.1);
	  }
	  else if(biggest.distance < 750.0f)
		  twist.linear.z = lzDelta.get_velocity(0.1);
	  else
		  twist.linear.z = lzDelta.get_velocity(0);

	  twist.linear.y = lyDelta.get_velocity((-(cx - 0.5) / 0.5));

	  twist.linear.x = lxDelta.get_velocity((-(cy - 0.5) / 0.5));

          float rotZ = biggest.zRot;
          if(rotZ > 1.5) 
             rotZ = rotZ - 3;
          if(rotZ < -1.5)
             rotZ = rotZ + 3;

	  twist.angular.z = azDelta.get_velocity(-rotZ * ( 0.66667 ));


	  if(abs(twist.linear.y) < 0.35)
	  {
	      lyDelta.get_velocity(0);
	      twist.linear.y = 0;
	  }

	  
          if(twist.linear.x < 0.25 && twist.linear.x > -0.35)
	  {
		  lxDelta.get_velocity(0);
		  twist.linear.x = 0;
	  }

	  if((altd > 1500 && twist.linear.z > 0)  || (altd < 300 && twist.linear.z < 0))
		twist.linear.z = 0;

	  zr = biggest.zRot;
          twist.linear.x /= 8;
          twist.linear.y /= 8;
  }



  twist.linear.y = max(-0.05, min(0.05, twist.linear.y));
  twist.linear.x = max(-0.05, min(0.05, twist.linear.x));

  vor = twist.linear.x > 0;
  zurueck = twist.linear.x < 0;
  links = twist.linear.y > 0;
  rechts = twist.linear.y < 0;

  pub.publish(twist);
  ostringstream ostr;
  ostr << "linear.x: " << twist.linear.x << endl;
  ostr << "linear.y: " << twist.linear.y << endl << endl;
  ostr << "linear.z " << twist.linear.z << endl;
  ostr << "angular.z " << twist.angular.z << endl;
  ostr << "zRot    : " << zr << endl;
  ostr << "altd: " << altd << endl;
  if(msg->tag_count > 0)
    ostr << "Distance: " << msg->tags[0].distance << endl;
  ROS_INFO(ostr.str().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_tag_bottom");

  sinceNotSeen = time(NULL);

  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.z = 0;
  ros::NodeHandle node_handle;
  pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
  ros::Subscriber sub = node_handle.subscribe("tags",1000, handleTag);

  ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, navdataUpdate);

  pub.publish(twist);

  ros::spin();
}
