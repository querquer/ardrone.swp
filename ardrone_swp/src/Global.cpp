#include "Global.h"
#include <time.h>

Cglobal* Cglobal::m_instance = 0;


Cglobal::Cglobal()
: altd(-1)
, vor(false)
, zurueck(false)
, links(false)
, rechts(false)
, hoch(false)
, runter(false)
, ges(0.05)
, end(false)
, seen(false)
, vx(0.0f)
, vy(0.0f)
, vz(0.0f)
, rotx(0.0f)
, roty(0.0f)
, sinceNotSeen(time(NULL))
, lastDir(-1)
, exsum(0.0f)
, eysum(0.0f)
, ezsum(0.0f)
, exold(0.0f)
, eyold(0.0f)
, ezold(0.0f)
, dxold(0.0f)
, dyold(0.0f)
, of("/home/ulrich/ros_workspace/ardrone_swp/Log.txt")
{
	gettimeofday(&sinceNoNavdataUpdate, NULL);
}

Cglobal& Cglobal::instance()
{
   if ( !m_instance )
      m_instance = new Cglobal();
   return *m_instance;
}

void Cglobal::destroy()
{
   if ( m_instance )
     delete m_instance;
   m_instance = 0;
}
