#include "Global.h"
#include <time.h>

Cglobal* Cglobal::m_instance = 0;


Cglobal::Cglobal()
: altd(-1)
, ges(0.05)
, end(false)
, seen(false)
, vx(0.0f)
, vy(0.0f)
, vz(0.0f)
, rotx(0.0f)
, roty(0.0f)
, sinceNotSeen(time(NULL))
, exold(0.0f)
, eyold(0.0f)
, begin(true)
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
