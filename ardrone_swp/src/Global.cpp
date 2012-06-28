#include "Global.h"


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
, widthB(160)
, heightB(120)
, widthF(320)
, heightF(240)
, seen(false)
, vx(0.0f)
, vy(0.0f)
, vz(0.0f)
, rotx(0.0f)
, roty(0.0f)
, lastSeen(0)
, lastDir(-1)
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
