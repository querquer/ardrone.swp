#include "ardrone_brown/Navdata.h"
#include "ros/ros.h"

bool info = false;
float batteryPercent1 = -1;
float rotX1 = -1;
float rotY1 = -1;
float rotZ1 = -1;
int altd1 = -1;
float vx1 = -1;
float vy1 = -1;
float vz1 = -1;
float tm1 = -1;

void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
  batteryPercent1 = navdata->batteryPercent;
rotX1 = navdata->rotX;
rotY1 = navdata->rotY;
rotZ1 = navdata->rotZ;

altd1 = navdata->altd;
vx1 = navdata->vx;
vy1 = navdata->vy;
vz1 = navdata->vz;
tm1 = navdata->tm;
  info = true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "startNavdata");

  ros::NodeHandle node_handle;

  ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, navdataUpdate); 

ros::Rate loop_rate(40);
  while(ros::ok())
  {
    ros::spinOnce();

    if(info)
    {
      std::ostringstream out;
      out << info << "\n" << batteryPercent1 << "\n" << rotX1 << " " << "\n" << rotY1 << "\n" << rotZ1;
      out <<  "\n" << vx1 <<  "\n" << vy1 << "\n" << vz1 <<  "\n" << tm1;
      out << "\n\n";
      ROS_INFO("navdata: %s", out.str().c_str());
    }
    info = false;
    loop_rate.sleep();
  }

  ros::spin();
}
