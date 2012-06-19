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

float vx;
float vy;
float vz;

int count = 0;


void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
     //sammeln!
	//ist nicht so gut, weil wir sonst während des Sammelns mit den alten Werten arbeiten
	//falls ein Ausreißer dabei ist, ist das nicht so dramatisch, weil auch nur einmal ein Twist-Objekt mit Ausreißer Daten gesendet werden würde
	altd = navdata->altd;
    vx = navdata->vx;
    vy = navdata->vy;
    vz = navdata->vz;
}

void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
  if(msg->tag_count == 0)   //Falls kein Tag erkannt wurde
  {
    if(seen)   //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
      sinceNotSeen = time(NULL);
    //if(time(NULL) - sinceNotSeen < 1)  //Falls ein Tag in der letzen Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
    if(time(NULL) - sinceNotSeen < 2)     //n.n.t
    {
      if(vor)
      {
    	  if(vx < 0)
    		  twist.linear.x = lxDelta.get_velocity(0.1 - 1.5 *  (vx / 4000));   //Drone ist außerhalb des Bereiches und fliegt in falsche Richtung
    	  else
    		  twist.linear.x = lxDelta.get_velocity(0.1);
      }
      if(zurueck)
      {
    	  if(vx > 0)
    		  twist.linear.x = lxDelta.get_velocity(-0.1 - 1.5 *  (vx / 4000));
    	  else
    		  twist.linear.x = lxDelta.get_velocity(-0.1);
      }
      if(links)
      {
    	  twist.linear.y = lyDelta.get_velocity(0.1);
    	  if(vy < 0)
    		  twist.linear.y = lyDelta.get_velocity(0.1 - 1.5 *  (vy / 4000));
    	  else
    		  twist.linear.y = lyDelta.get_velocity(0.1);
      }
      if(rechts)
      {
    	  if(vy > 0)
    		  twist.linear.y = lyDelta.get_velocity(-0.1 - 1.5 *  (vy / 4000));
    	  else
    		  twist.linear.y = lyDelta.get_velocity(-0.1);
      }
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
  else //Falls mindestens ein Tag gesehen wurde
  {
	  seen = true;
	 // int width = msg->image_width;
	 // int height = msg->image_height;
	  int width = 160;
	  int height = 120;

	  ar_recog::Tag biggest = msg->tags[0];

	  for(int i = 0; i < msg->tag_count; ++i)  //Suche das größte Tag
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
	  cx = cx / 4.0 / width;   //cx: 0 Tag ist am oberen Bildrand, 0.5 Tag ist in der Mitte, 1 Tag ist am unteren Bildrand
	  cy = cy / 4.0 / height;  //cy: 0 Tag ist liks, 0.5 Tag ist in der Mitte, 1 Tag ist rechts



	  if(biggest.distance > 1650.0f)
	  {
	    twist.linear.z = lzDelta.get_velocity(-0.1);
	  }
	  else if(biggest.distance < 1500.0f)                 //Versuche etwa bei einer Höhe von 1,5 m zu schweben
		  twist.linear.z = lzDelta.get_velocity(0.3);
	  else
		  twist.linear.z = lzDelta.get_velocity(0);

	  twist.linear.y = lyDelta.get_velocity((-(cx - 0.5) / 4));

      //twist.linear.y = lyDelta.get_velocity(-(cx - 0.5) * 1.1);

      twist.linear.x = lxDelta.get_velocity(-(cy - 0.5) / 4);

      twist.linear.y = max(-0.08, min(0.08, twist.linear.y));
      twist.linear.x = max(-0.08, min(0.08, twist.linear.x));   // Geschwindigkeit von in x un y Richtung steigt linear mit der Entfernung zum Mittelpunkt
                                                                // 0.08 ist das Maximum

      float rotZ = biggest.zRot;    //versuche längs zum Tag zu sein, dabei enteder vorne oder hinten(was näher dran ist)
      if(rotZ > 1.5)
         rotZ = rotZ - 3;
      if(rotZ < -1.5)
         rotZ = rotZ + 3;

	  twist.angular.z = azDelta.get_velocity(-rotZ * ( 0.66667 ));  // Drehe dich in Abhängigkeit von der z Rotation des Tags


	  /*if(abs(twist.linear.y) < 0.35)
	  {
	      lyDelta.get_velocity(0);
	      twist.linear.y = 0;
	  }

	  
          if(twist.linear.x < 0.25 && twist.linear.x > -0.35)
	  {
		  lxDelta.get_velocity(0);
		  twist.linear.x = 0;
	  }*/

	  if((altd > 1700 && twist.linear.z > 0)  || (altd < 300 && twist.linear.z < 0))  //Fliege nicht höher als 1,7m und nicht niedriger als 0,3m
		twist.linear.z = 0;

          //twist.linear.x /= 8;
          //twist.linear.y /= 8;
  }



  //twist.linear.y = max(-0.1, min(0.1, twist.linear.y));
  //twist.linear.x = max(-0.1, min(0.1, twist.linear.x));

  vor = twist.linear.x > 0;         //In welche Richtung willst du fiegen? (wichtig, falls das Tag verloren geht)
  zurueck = twist.linear.x < 0;
  links = twist.linear.y > 0;
  rechts = twist.linear.y < 0;

  //twist.linear.x = 0;
  //twist.linear.y = 0;

  if((vx > 0 && twist.linear.x < 0) || (vx < 0 && twist.linear.x > 0))   //Falls du in die falsche Richtung fliegst, verstärke deine Bewegung
  {
     twist.linear.x += -1.5 * (vx / 4000);
  }
  if((vy > 0 && twist.linear.y < 0) || (vy < 0 && twist.linear.y > 0))
  {
     twist.linear.y += -1.5 * (vy / 4000);
  } 
  
  if(twist.linear.x * vx > 0 && abs(twist.linear.x) * 5000 < abs(vx))  //Wenn Rüdiger schneller nach vorne oder hinten fliegt, als er soll, bremse
  {
	  twist.linear.x -= vx / 5000;
  }

  if(twist.linear.y * vy > 0 && abs(twist.linear.y) * 5000 < abs(vy))  //Wenn Rüdiger schneller nach rechts oder links fliegt, als er soll, bremse
  {
	  twist.linear.y -= vy / 5000;
  }

  pub.publish(twist);
  ostringstream ostr;
  ostr << "\n\nlinear.x: " << twist.linear.x << endl;
  ostr << "linear.y: " << twist.linear.y << endl;
  ostr << "linear.z " << twist.linear.z << endl;
  ostr << "angular.z " << twist.angular.z << endl;
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

  vx = 0;
  vy = 0;
  vz = 0;

  ros::NodeHandle node_handle;
  pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
  ros::Subscriber sub = node_handle.subscribe("tags",1000, handleTag);

  ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, navdataUpdate);

  pub.publish(twist);

  ros::spin();
}
