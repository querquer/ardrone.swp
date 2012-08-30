#ifndef GLOBAL_H
#define GLOBAL_H

#include "std_includes.h"

#include <boost/noncopyable.hpp>

/*#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "Delta.h"

#include <time.h>

#include <sys/time.h>*/

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

      geometry_msgs::Twist twist_old;

      ros::Publisher pub;   ///< publisher zum publishen des Twist Objekts

      int altd; ///< Höhe

      time_t sinceNotSeen; ///< Zeit seit dem das Tag das letzte mal gesehen wurde

      struct timeval sinceNoNavdataUpdate;

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

      //Bildgrößen
      static const int widthB = 160;  ///< ...
      static const int heightB = 120;
      static const int widthF = 320;
      static const int heightF = 240;



      //Regelungsparameter
      static const float b_mmPs2twistx = 0.0002f; //weil Drone in  x Richtung max 5m/s fliegt
  	  static const float b_mmPs2twisty = 0.0002f;

      static const float f_mmPs2twistx = 0.0002f;
  	  static const float f_mmPs2twisty = 0.0002f;

      static const float l_mmPs2twistx = 0.00015f;
  	  static const float l_mmPs2twisty = 0.00025f;

  	  static const float b_Kpx = 2.5f;
  	  static const float b_Kpy = 1.5f;

  	  static const float f_Kpx = 1.5f;
  	  static const float f_Kpy = 2.5f;

  	  static const float l_Kpx = 1.7f;
  	  static const float l_Kpy = 1.0f;

  	  static const float Ta = 0.05555555f; ///< 1/18, weil 18 Aufrufe pro Sekunde

  	  static const float b_Kdx = 0.05f;
  	  static const float b_Kdy = 0.05f;

  	  static const float f_Kdx = 0.05f;
  	  static const float f_Kdy = 0.05f;

  	  static const float l_Kdx = 0.05f;
  	  static const float l_Kdy = 0.05f;


      float exsum;
      float eysum;
      float ezsum;

      float exold;
      float eyold;
      float ezold;

      float dxold;
      float dyold;

      std::ofstream of;
};



#endif //GLOBAL_H
