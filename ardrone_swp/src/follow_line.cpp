#include "std_includes.h"
#include "ardrone_swp/LinePos.h"

using namespace std;

void handleLine(const ardrone_swp::LinePos::ConstPtr& msg)
{
	ostringstream ostr;
  if(msg->x == Cglobal::instance().widthB/2 && msg->y == Cglobal::instance().heightB/2)
  {
    //keine Linie wird erkannt
	Cglobal::instance().twist.linear.x = 0;
	Cglobal::instance().twist.linear.y = 0;
	Cglobal::instance().twist.linear.z = 0;
	Cglobal::instance().twist.angular.z = 0;
  }
  else
  {
  //twist.angular.z = msg->angle / 9000;

  float z = 0;
  if(abs(msg->angle) > 7000)
    z = msg->angle > 0 ? 0.3 : -0.3;
  else if(abs(msg->angle) > 4000)
    z = msg->angle > 0 ? 0.2 : -0.2;
  else if(abs(msg->angle) > 500)
    z = msg->angle > 0 ? 0.1 : -0.1;

  Cglobal::instance().twist.angular.z = Cglobal::instance().azDelta.get_velocity(z);

  //Fliege immer nach vorne, wenn Linie erkannt wurde
  Cglobal::instance().twist.linear.x = Cglobal::instance().lxDelta.get_velocity(0.05);

  if(msg->x > Cglobal::instance().widthB/2 + 5)
	  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(0.1);
  else if(msg->y < Cglobal::instance().heightB/2 - 5)
	  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(-0.1);
  else
	  Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(0);

  if(Cglobal::instance().altd > 900)
	  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(-0.2);
  else if(Cglobal::instance().altd < 800)
	  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(0.2);
  else
	  Cglobal::instance().twist.linear.z = Cglobal::instance().lzDelta.get_velocity(0);

  ostr << "x: " << msg->x << endl;
  ostr << "y: " << msg->y << endl;
  }

  ostr << "linear.x: " << Cglobal::instance().twist.linear.x << endl;
  ostr << "linear.y: " << Cglobal::instance().twist.linear.y << endl << endl;
  ostr << "linear.z " << Cglobal::instance().twist.linear.z << endl;
  ostr << "angular.z " << Cglobal::instance().twist.angular.z << endl;
  ostr << "altd: " << Cglobal::instance().altd << endl;
  ROS_INFO(ostr.str().c_str());
  Cglobal::instance().pub.publish(Cglobal::instance().twist);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_line2");

  ros::NodeHandle node_handle;
  Cglobal::instance().pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = node_handle.subscribe("LinePos",1000, handleLine);

  ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, Math::navdataUpdate);

  while(!Cglobal::instance().end && ros::ok())
  {
	  ros::spinOnce();
  }

  Cglobal::destroy();
}
