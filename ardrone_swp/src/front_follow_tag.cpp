#include "std_includes.h"

using namespace std;

void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
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
	  if(Cglobal::instance().hoch)
		  Cglobal::instance().twist.linear.y = Cglobal::instance().lzDelta.get_velocity(0.1);
	  if(Cglobal::instance().runter)
		  Cglobal::instance().twist.linear.y = Cglobal::instance().lzDelta.get_velocity(-0.1);
	}
	else //Versuche ruhig in Luft zu stehen
	{
	  	Cglobal::instance().twist.linear.x = 0;
	  	Cglobal::instance().twist.linear.z = 0;
	  	Cglobal::instance().twist.linear.y = 0;

	    if(time(NULL) - Cglobal::instance().lastSeen > 0.5)
	    	Cglobal::instance().twist.angular.z = 0.5 * Cglobal::instance().lastDir;
	}
	Cglobal::instance().seen = false;
  }
  else //Falls mindestens ein Tag gesehen wurde
  {
  Cglobal::instance().seen = true;
  Cglobal::instance().lastSeen = time(NULL);
  ar_recog::Tag biggest = msg->tags[0];

  for(int i = 0; i < msg->tag_count; ++i)
  {
    if(msg->tags[i].diameter > biggest.diameter)
      biggest = msg->tags[i];
  }

  float cx = 0;
  float cy = 0;
  Math::centerFront(biggest,cx,cy);

  float stopping_dist = 600.0;
  float dist = (biggest.distance -stopping_dist) / stopping_dist;
  float dist_vel = Cglobal::instance().lxDelta.get_velocity(dist);

  if(abs(dist) < 0.25)
	  Cglobal::instance().twist.linear.x = dist_vel * 0.25; //if we are close enough to the stopping distance, just try to stop
  else
	  Cglobal::instance().twist.linear.x = dist * 0.25; //otherwise try to move within stopping_dist

  Cglobal::instance().twist.linear.x = max(-0.05, min(0.05, Cglobal::instance().twist.linear.x));

  //try to face perpendicular to the tag
  float yRot_velocity = Cglobal::instance().azDelta.get_velocity(biggest.yRot);
  Cglobal::instance().twist.angular.z = yRot_velocity * 0.5;

  Cglobal::instance().twist.angular.z = max(-0.5, min(0.5, Cglobal::instance().twist.angular.z));
  //if (abs(biggest.yRot) < 0.2)
	//twist.linear.y = yRot_velocity * 0.25; //if we are mostly facing perpendicular, just try to stay still
  //else
	//twist.linear.y = biggest.yRot * 0.125; //otherwise, rotate towards being in front of the tag



  //rotate to face the tag
  Cglobal::instance().twist.linear.y = (-(cx - 0.5)/0.5) * 0.2;
  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(Cglobal::instance().twist.linear.y);
  Cglobal::instance().twist.linear.y = max(-0.05, min(0.05, Cglobal::instance().twist.linear.y));

  if (Cglobal::instance().twist.angular.z < 0)
	  Cglobal::instance().lastDir = -1;
  else
	  Cglobal::instance().lastDir = 1;


  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity((-(cy - 0.5) / 0.5));
  if(Cglobal::instance().twist.linear.z > 0.5)
	  Cglobal::instance().twist.linear.z = 0.1;
  else if(Cglobal::instance().twist.linear.z < -0.4)
	  Cglobal::instance().twist.linear.z = -0.1;
  else
	  Cglobal::instance().twist.linear.z = 0;
  Cglobal::instance().twist.linear.z = max(-0.1, min(0.3, Cglobal::instance().twist.linear.z));

  if((Cglobal::instance().altd > 1300 && Cglobal::instance().twist.linear.z > 0)  || (Cglobal::instance().altd < 500 && Cglobal::instance().twist.linear.z < 0))
	  Cglobal::instance().twist.linear.z = 0;
  }
  ostringstream ostr;

  Keyboard::control();

  Cglobal::instance().vor = Cglobal::instance().twist.linear.x > 0;
  Cglobal::instance().zurueck = Cglobal::instance().twist.linear.x < 0;
  Cglobal::instance().links = Cglobal::instance().twist.linear.y > 0;
  Cglobal::instance().rechts = Cglobal::instance().twist.linear.y < 0;
  Cglobal::instance().hoch = Cglobal::instance().twist.linear.z > 0;
  Cglobal::instance().runter = Cglobal::instance().twist.linear.z < 0;

  Cglobal::instance().pub.publish(Cglobal::instance().twist);
  ostr << "ges: " << Cglobal::instance().ges << endl;
  ostr << "linear.x: " << Cglobal::instance().twist.linear.x << endl;
  ostr << "linear.y: " << Cglobal::instance().twist.linear.y << endl;
  ostr << "linear.z: " << Cglobal::instance().twist.linear.z << endl;
  ostr << "angular.z: " << Cglobal::instance().twist.angular.z << endl << endl;
  if(msg->tag_count > 0)
    ostr << "Erkannt!!\nDistance: " << msg->tags[0].distance << endl;
  ROS_INFO(ostr.str().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_tag");

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
