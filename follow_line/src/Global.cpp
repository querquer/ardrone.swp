#include "Global.h"


Cglobal* Cglobal::m_instance = 0;


Cglobal::Cglobal()
: altd(-1)
, vor(false)
, zurueck(false)
, links(false)
, rechts(false)
, ges(0.05)
, end(false)
, width(160)
, height(120)
, seen(false)
, vx(0.0f)
, vy(0.0f)
, vz(0.0f)
, rotx(0.0f)
, roty(0.0f)
{

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