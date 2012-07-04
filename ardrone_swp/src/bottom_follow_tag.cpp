#include "std_includes.h"

using namespace std;

/**
 * @file bottom_follow_tag.cpp
 * @brief Applikation zur Tagvervolgung mit der unteren Kamera
 */



/** @brief subscriber handler für die Nachricht tags
 */
void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
  ostringstream ostr;
  if(msg->tag_count == 0)   //Falls kein Tag erkannt wurde
  {
    if(Cglobal::instance().seen)   //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
    	Cglobal::instance().sinceNotSeen = time(NULL);
    //if(time(NULL) - Cglobal::instance().sinceNotSeen < 1)  //Falls ein Tag in der letzen Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
    if(time(NULL) - Cglobal::instance().sinceNotSeen < 2)     //n.n.t
    {
      if(Cglobal::instance().vor)
    	  Cglobal::instance().twist.linear.x = Cglobal::instance().lxDelta.get_velocity(0.1);
      if(Cglobal::instance().zurueck)
    	  Cglobal::instance().twist.linear.x = Cglobal::instance().lxDelta.get_velocity(-0.1);
      if(Cglobal::instance().links)
    	  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(0.1);
      if(Cglobal::instance().rechts)
    	  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(-0.1);
    }
    else //Versuche ruhig in Luft zu stehen
    {
    	Cglobal::instance().twist.linear.x = Cglobal::instance().lxDelta.get_velocity(0);
    	Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(0);
    	Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(0);
    	Cglobal::instance().twist.angular.z = Cglobal::instance().azDelta.get_velocity(0);
    }
    Cglobal::instance().seen = false;
  }
  else //Falls mindestens ein Tag gesehen wurde
  {
	  Cglobal::instance().seen = true;
	  ar_recog::Tag biggest = msg->tags[0];

	  for(unsigned int i = 0; i < msg->tag_count; ++i)  //Suche das größte Tag
	  {
	    if(msg->tags[i].diameter > biggest.diameter)
	      biggest = msg->tags[i];
	  }

	  float cx = 0;
	  float cy = 0;
	  /*
	   * Bestimme die Mittelpunkte
	   */
	  Math::centerBottom(biggest, cx, cy);

	  ostr << "cy      :" << cy << endl;
	  ostr << "cx     :" << cx << endl;

	  /*
	   etwa bei einer Höhe von 1,5 m schweben
	   */
	  if(biggest.distance > 1650.0f)
	  {
		  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(-0.1);
	  }
	  else if(biggest.distance < 1500.0f)
		  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(0.3);
	  else
		  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(0);

	  /*
	   * Geschwindigkeit in x un y Richtung steigt linear mit der Entfernung zum Mittelpunkt
	   * 0.08 ist das Maximum
	   */
	  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(max(-0.08, min(0.08, -(cx - 0.5) / 4)));
	  Cglobal::instance().twist.linear.x = Cglobal::instance().lxDelta.get_velocity(max(-0.08, min(0.08, -(cy - 0.5) / 4)));

      /*float rotZ = biggest.zRot;    //versuche längs zum Tag zu sein, dabei entweder vorne oder hinten(was näher dran ist)
      if(rotZ > 1.5)
         rotZ = rotZ - 3;
      if(rotZ < -1.5)
         rotZ = rotZ + 3;*/

	  //twist.angular.z = azDelta.get_velocity(-biggest.zRot * ( 0.66667 ));  // Drehe dich in Abhängigkeit von der z Rotation des Tags
	  Cglobal::instance().twist.angular.z = Cglobal::instance().azDelta.get_velocity(max(-1.5, min(1.5, biggest.zRot)) * ( 0.666667 ));
  }

  //Fliege nicht höher als 1,7m und nicht niedriger als 0,3m
  if((Cglobal::instance().altd > 1700 && Cglobal::instance().twist.linear.z > 0)
		  || (Cglobal::instance().altd < 300 && Cglobal::instance().twist.linear.z < 0))
	  Cglobal::instance().twist.linear.z = 0;

  //speichern in welche Richtung geflogen werden soll (wichtig, falls das Tag verloren geht)
  Cglobal::instance().vor = Cglobal::instance().twist.linear.x > 0;
  Cglobal::instance().zurueck = Cglobal::instance().twist.linear.x < 0;
  Cglobal::instance().links = Cglobal::instance().twist.linear.y > 0;
  Cglobal::instance().rechts = Cglobal::instance().twist.linear.y < 0;

  //Falls die Drone in die falsche Richtung flieg, verstärke die Bewegung
  if((Cglobal::instance().vx > 0 && Cglobal::instance().twist.linear.x < 0)
		  || (Cglobal::instance().vx < 0 && Cglobal::instance().twist.linear.x > 0))
  {
	  Cglobal::instance().twist.linear.x -= 1 * (Cglobal::instance().vx / 4000);
  }
  if((Cglobal::instance().vy > 0 && Cglobal::instance().twist.linear.y < 0)
		  || (Cglobal::instance().vy < 0 && Cglobal::instance().twist.linear.y > 0))
  {
	  Cglobal::instance().twist.linear.y -= 1 * (Cglobal::instance().vy / 4000);
  } 
  
  //Wenn die Drone schneller nach vorne oder hinten fliegt, als sie soll, bremse
  if(Cglobal::instance().twist.linear.x * Cglobal::instance().vx > 0
		  && abs(Cglobal::instance().twist.linear.x) * 5000 < abs(Cglobal::instance().vx))
  {
	  Cglobal::instance().twist.linear.x -= 0.7 * (Cglobal::instance().vx / 5000);
  }

  //Wenn die Drone schneller nach rechts oder links fliegt, als sie soll, bremse
  if(Cglobal::instance().twist.linear.y * Cglobal::instance().vy > 0
		  && abs(Cglobal::instance().twist.linear.y) * 5000 < abs(Cglobal::instance().vy))
  {
	  Cglobal::instance().twist.linear.y -= 0.7 * (Cglobal::instance().vy / 5000);
  }


  Keyboard::control();  //Falls Tastatureingaben zum Steuern möglich sein sollen


  Cglobal::instance().pub.publish(Cglobal::instance().twist);


  ostr << "\n\nlinear.x: " << Cglobal::instance().twist.linear.x << endl;
  ostr << "linear.y: " << Cglobal::instance().twist.linear.y << endl;
  ostr << "linear.z " << Cglobal::instance().twist.linear.z << endl;
  ostr << "angular.z " << Cglobal::instance().twist.angular.z << endl;
  ostr << "altd: " << Cglobal::instance().altd << endl;
  if(msg->tag_count > 0)
    ostr << "Distance: " << msg->tags[0].distance << endl;

  ROS_INFO(ostr.str().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_tag_bottom");

  Cglobal::instance().sinceNotSeen = time(NULL);

  ros::NodeHandle node_handle;
  Cglobal::instance().pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = node_handle.subscribe("tags",1000, handleTag);

  ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, Math::navdataUpdate);

  while(!Cglobal::instance().end && ros::ok())
  {
	  ros::spinOnce();
  }
  Cglobal::destroy();
}
