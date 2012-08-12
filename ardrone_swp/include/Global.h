#ifndef GLOBAL_H
#define GLOBAL_H

#include <boost/noncopyable.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "Delta.h"

#include <time.h>

/* @brief Klasse, die in Form eines Singletons alle globalen Variablen enthält
 *
 */
class Cglobal : boost::noncopyable
{
private:
      static Cglobal* m_instance;

      Cglobal();
      ~Cglobal() {}
   public:
      static Cglobal& instance();
      static void destroy();

   public:
      geometry_msgs::Twist twist; ///< Zur Ansteuerung der Drone

      ros::Publisher pub;   ///< publisher zum publishen des Twist Objekts

      int altd; ///< Höhe

      bool vor;  ///< will die Drone nach vorne fliegen?
      bool zurueck;
      bool links;
      bool rechts;
      bool hoch;
      bool runter;

      time_t sinceNotSeen; ///< Zeit seit dem das Tag das letzte mal gesehen wurde

      float lastDir;

      bool seen;

      Delta lxDelta;
      Delta lyDelta;
      Delta lzDelta;
      Delta azDelta;

      float vx;
      float vy;
      float vz;
      float roty;
      float rotx;

      float ges;

      bool end;

      static const int widthB = 160;
      static const int heightB = 120;
      static const int widthF = 320;
      static const int heightF = 240;

      float exsum;
      float eysum;
      float ezsum;

      float exold;
      float eyold;
      float ezold;
};



#endif //GLOBAL_H
