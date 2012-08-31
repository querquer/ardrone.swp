/**
 * @file Log.cpp
 * @brief Applikation zum Loggen der Navdata, Twist, Tags
 *
 * Option, die beim Aufruf übergeben werden können:
 *
 * n : Navdata loggen
 *
 * t : Tags loggen
 *
 * w : gesentete Twist-Objekte loggen
 *
 * Beispielsaufruf: rosrun ardrone_swp Log n w t
 *
 * ->loggt Navtada, Tags und Twist-Objekte
 *
 * wird in die Ornder: ardrone_swp/Log/ mit den Unterorndern LogNavdata, LogTags, LogTwist geschrieben
 *
 * Die Dateinahmen sind das aktuelle Datum
 */

#include "ardrone_brown/Navdata.h"
#include "ros/ros.h"
#include "ar_recog/Tags.h"
#include "ar_recog/Tag.h"
#include "geometry_msgs/Twist.h"

#include <time.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <sys/time.h>

#include <boost/filesystem.hpp>


using namespace std;

ofstream logNavdata;
ofstream logTags;
ofstream logTwist;

/** @brief schreibt die Tag-Informationen in die entsprechende Datei
 *
 */
void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
  time_t t;
  time(&t);
  logTags << ctime(&t) << endl;
  logTags << "Anzahl der Tags: " << msg->tag_count << endl;

  if(msg->tag_count > 0)
  {
    logTags  << "image_width: " << msg->image_width << endl;
    logTags  << "image_height: " << msg->image_height << endl;
    logTags  << "angle_of_view: " << msg->angle_of_view << endl;

    for(size_t i = 0; i < msg->tag_count; ++i)
    {
      logTags << "ID: " << msg->tags[i].id << endl;
      logTags << "cf: " << msg->tags[i].cf << endl;
      logTags << "cf: " << msg->tags[i].cf << endl;
      logTags << "x: " << msg->tags[i].x << endl;
      logTags << "y: " << msg->tags[i].y << endl;
      logTags << "distance: " << msg->tags[i].distance << endl;
      logTags << "diameter: " << msg->tags[i].diameter << endl;
      logTags << "xRot: " << msg->tags[i].xRot << endl;
      logTags << "yRot: " << msg->tags[i].yRot << endl;
      logTags << "zRot: " << msg->tags[i].zRot << endl;
      logTags << "xMetric: " << msg->tags[i].xMetric << endl;
      logTags << "yMetric: " << msg->tags[i].yMetric << endl;
      logTags << "zMetric: " << msg->tags[i].zMetric << endl;
    }
    logTags << endl;
  }
}
/** @brief schreibt die Navdata-Informationen in die entsprechende Datei
 *
 */
void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
  time_t t;
  time(&t);
  logNavdata << ctime(&t) << endl;
  logNavdata << "Batterie: " << navdata->batteryPercent << endl;
  logNavdata << "RotX: " << navdata->rotX << endl;
  logNavdata << "RotY: " << navdata->rotY << endl;
  logNavdata << "RotZ: " << navdata->rotZ << endl;
  logNavdata << "Höhe: " << navdata->altd << endl;
  logNavdata << "vx: " << navdata->vx << endl;
  logNavdata << "vy: " << navdata->vy << endl;
  logNavdata << "vz: " << navdata->vz << endl;
  logNavdata << "Zeit seit Start: " << navdata->tm << endl << endl;
}
/** @brief schreibt die Twist-Informationen in die entsprechende Datei
 *
 */
void handleTwist(const geometry_msgs::TwistConstPtr &msg)
{
  time_t t;
  time(&t);
  logTwist << ctime(&t) << endl;
  logTwist << "linear.x: " << msg->linear.x << endl;
  logTwist << "linear.y: " << msg->linear.y << endl;
  logTwist << "linear.z: " << msg->linear.z << endl;
  logTwist << "angular.x: " << msg->angular.x << endl;
  logTwist << "angular.y: " << msg->angular.y << endl;
  logTwist << "angular.z: " << msg->angular.z << endl << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LogNavdata");

  ros::NodeHandle node_handle;
  bool n = false; //navdata
  bool tag = false; //tags
  bool w = false; //twist
  time_t t;
  time(&t);
  ros::Subscriber navdataSub;
  ros::Subscriber tagsSub;
  ros::Subscriber twistSub;

  boost::filesystem::path p(argv[0]);
  for(int i = 0; i < argc; ++i)
  {
    if(!n && argv[i][0] == 'n')
    {
      ostringstream ost;
      ost << "/Log/LogNavdata/";
      ost << ctime(&t);

      boost::filesystem::path p2(p.parent_path().parent_path().string() + ost.str());
      logNavdata.open(p2.string().c_str());
      navdataSub = node_handle.subscribe("/ardrone/navdata", 1000, navdataUpdate);
      n = true;
    }
    else if(!tag && argv[i][0] == 't')
    {
      ostringstream ost;
      ost << "/Log/LogTags/";
      ost << ctime(&t);

      boost::filesystem::path p2(p.parent_path().parent_path().string() + ost.str());
      logTags.open(p2.string().c_str());
      tagsSub = node_handle.subscribe("tags",1000, handleTag);
      tag = true;
    }
    else if(!w && argv[i][0] == 'w')
    {
      ostringstream ost;
      ost << "/Log/LogTwist/";
      ost << ctime(&t);

      boost::filesystem::path p2(p.parent_path().parent_path().string() + ost.str());
      logTwist.open(p2.string().c_str());
      twistSub = node_handle.subscribe("/cmd_vel", 1, handleTwist);
      w = true;
    }
  }  
  ros::spin();
}

