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

#include <ncurses.h>

using namespace std;


geometry_msgs::Twist twist;


ros::Publisher pub;
int altd = -1;

time_t lastSeen;
float lastDir;

Delta lxDelta;
Delta lyDelta;
Delta lzDelta;

Delta azDelta;

float ges = 0.05;

void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
	altd = navdata->altd;
}
void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
	float cyd;
  if(msg->tag_count == 0)
  {
    twist.linear.x = lxDelta.get_velocity(0);
    twist.linear.y = lyDelta.get_velocity(0);
    twist.linear.z = lzDelta.get_velocity(0);
    if(time(NULL) - lastSeen > 0.5)
      twist.angular.z = 0.5 * lastDir;
  }
  else
  {
  lastSeen = time(NULL);

  //int width = msg->image_width;
  //int height = msg->image_height;
  int width = 320;
  int height = 240;


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


  float stopping_dist = 600.0;
  float dist = (biggest.distance -stopping_dist) / stopping_dist;
  float dist_vel = lxDelta.get_velocity(dist);

  if(abs(dist) < 0.25)
    twist.linear.x = dist_vel * 0.25; //if we are close enough to the stopping distance, just try to stop
  else
	twist.linear.x = dist * 0.25; //otherwise try to move within stopping_dist

  twist.linear.x = max(-0.05, min(0.05, twist.linear.x));

  //try to face perpendicular to the tag
  float yRot_velocity = azDelta.get_velocity(biggest.yRot);
  twist.angular.z = yRot_velocity * 0.5;

  twist.angular.z = max(-0.5, min(0.5, twist.angular.z));
  //if (abs(biggest.yRot) < 0.2)
	//twist.linear.y = yRot_velocity * 0.25; //if we are mostly facing perpendicular, just try to stay still
  //else
	//twist.linear.y = biggest.yRot * 0.125; //otherwise, rotate towards being in front of the tag



  //rotate to face the tag
  twist.linear.y = (-(cx - 0.5)/0.5) * 0.2;
  twist.linear.y = lyDelta.get_velocity(twist.linear.y);
  twist.linear.y = max(-0.05, min(0.05, twist.linear.y));

  cyd = (-(cy - 0.5) / 0.5);

  if (twist.angular.z < 0)
	lastDir = -1;
  else
	lastDir = 1;


  twist.linear.z = lzDelta.get_velocity((-(cy - 0.5) / 0.5));
  if(twist.linear.z > 0.5)
	twist.linear.z = 0.1;
  else if(twist.linear.z < -0.4)
	twist.linear.z = -0.1;
  else
	twist.linear.z = 0;
  //twist.linear.z = max(-0.1, min(0.1, twist.linear.z));

  //if((altd > 1300 && twist.linear.z > 0)  || (altd < 500 && twist.linear.z < 0))
  //  twist.linear.z = 0;
  }
  ostringstream ostr;

  //if(kbhit()) // Nur wenn auch eine Taste gedrückt ist
  //{
      char c = getch(); // Muss auf keine Eingabe warten, Taste ist bereits gedrückt
      switch(c)
      {
      case 'w':
    	  twist.linear.x = 1 * ges;
    	  ostr << "/nw pressed" << endl;
      case 's':
    	  twist.linear.x = -1 * ges;
    	  ostr << "s pressed" << endl;
      case 'a':
    	  twist.linear.y = 1 * ges;
    	  ostr << "a pressed" << endl;
      case 'd':
    	  twist.linear.y = -1 * ges;
    	  ostr << "d pressed" << endl;
      case 'q':
    	  twist.angular.z = -1 * ges;
    	  ostr << "q pressed" << endl;
      case 'e':
    	  twist.angular.z = 1 * ges;
    	  ostr << "e pressed" << endl;
      case 'o':
    	  twist.linear.z = 1 * ges;
    	  ostr << "o pressed" << endl;
      case 'l':
    	  twist.linear.z = -1 * ges;
    	  ostr << "l pressed" << endl;
      case 'u':
    	  ges += 0.1*ges;
    	  ostr << "u pressed" << endl;
      case 'j':
    	  ges += -0.1*ges;
    	  ostr << "l pressed" << endl;
      }
  //}



  pub.publish(twist);
  ostr << "ges: " << ges << endl;
  ostr << "linear.x: " << twist.linear.x << endl;
  ostr << "linear.y: " << twist.linear.y << endl;
  ostr << "Y bew:    " << cyd << endl;
  ostr << "linear.z: " << twist.linear.z << endl;
  ostr << "angular.z: " << twist.angular.z << endl << endl;
  if(msg->tag_count > 0)
    ostr << "Erkannt!!\nDistance: " << msg->tags[0].distance << endl;
  ROS_INFO(ostr.str().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_tag");
	initscr();			/* Start curses mode 		*/
	raw();				/* Line buffering disabled	*/
	//keypad(stdscr, TRUE);		/* We get F1, F2 etc..		*/
	noecho();			/* Don't echo() while we do getch */
  lastSeen = 0;
  lastDir = -1;
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
	endwin();			/* End curses mode		  */
}
