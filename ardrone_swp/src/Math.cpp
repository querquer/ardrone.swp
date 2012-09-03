#include "Math.h"
#include "Global.h"
#include "ardrone_brown/Navdata.h"


namespace
{
	void center(const ar_recog::Tag& tag, float& cx, float& cy, float dx, float dy, float width, float height);
	float grad2rad(float a);
}


namespace Math
{

/** @brief Berechnung der Anzahl der Pixel für die Bodenkamera, um die das Tag aufgrund der Rotation verschoben wurde
 *
 *
 *
 * @param	x	Verschiebung in x Richung
 * @param	y	Verschiebung in y Richung
 *
 *
 * <img src="/home/ulrich/ros_workspace/ardrone_swp/Bilder/Formel1.jpg" alt="Schema">
 * <img src="/home/ulrich/ros_workspace/ardrone_swp/Bilder/bottom_diff.jpg" alt="Schema">
 * <img src="/home/ulrich/ros_workspace/ardrone_swp/Bilder/Formel2.jpg" alt="Schema">
 * <img src="/home/ulrich/ros_workspace/ardrone_swp/Bilder/Kreis.jpg" alt="Schema">
 * <img src="/home/ulrich/ros_workspace/ardrone_swp/Bilder/Formel3.jpg" alt="Schema">
 */
void pixelDiffBottom(float& x, float& y)
{
	//Formel: siehe Dokumentation
	//0.5585 (in rad) = 32° (in grad)
	y = (200.0f * tan(Cglobal::instance().roty)) / ((tan(0.5585f + Cglobal::instance().roty) + tan(0.5585f - Cglobal::instance().roty)));
	x = (200.0f * tan(Cglobal::instance().rotx)) / ((tan(0.5585f + Cglobal::instance().rotx) + tan(0.5585f - Cglobal::instance().rotx)));

	//Der berechnete Wert ist zu klein, wird er mit 1,333 multipliziert stimmt es in etwa
	y *= 1.333f;
	x *= 1.333f;
}


/** @brief Berechnung der Anzahl der Pixel für die Frontkamera, um die das Tag aufgrund der Rotation verschoben wurde
 *
 * @param	x	Verschiebung in x Richung
 * @param	y	Verschiebung in y Richung
 *
 *
 * <img src="/home/ulrich/ros_workspace/ardrone_swp/Bilder/betta.jpg" alt="Schema">
 */
void pixelDiffFront(float& x, float& y)
{

	y = 0.5 * tan(Cglobal::instance().roty) * 0.948965 * Cglobal::instance().heightF;  //tan(43.5)~= 0.948965
	y *= 2.3f;
	//x = 0.5 * tan(Cglobal::instance().rotx) * 0.948965 * Cglobal::instance().widthF;
	x = 0;
}

/** @brief Berechnung für die Bodenkamera aus den Eckpunkten, wo sich das Tag bezüglich des Bildes befindet

	cy:

	 0:     Tag ist am oberen Bildrand,

	 0.5:   Tag ist in der Mitte(Höhe),

	 1:     Tag ist am unteren Bildrand

	cx:

	 0:	 	Tag ist am linken Bildrand,

	 0.5:	Tag ist in der Mitte(Breite),

	 1:		Tag ist am rechten Bildrend
*/
void centerBottom(const ar_recog::Tag& tag, float& cx, float& cy)
{
	float dx, dy;
	pixelDiffBottom(dx,dy);
	center(tag, cx, cy, dx, dy, Cglobal::instance().widthB, Cglobal::instance().heightB);
	cx -= 0.05f;
}
/** @brief Berechnung für die Front	kamera aus den Eckpunkten, wo sich das Tag bezüglich des Bildes befindet

	cy:

	 0:     Tag ist am oberen Bildrand,

	 0.5:   Tag ist in der Mitte(Höhe),

	 1:     Tag ist am unteren Bildrand

	cx:

	 0:	 	Tag ist am linken Bildrand,

	 0.5:	Tag ist in der Mitte(Breite),

	 1:		Tag ist am rechten Bildrend
*/
void centerFront(const ar_recog::Tag& tag, float& cx, float& cy)
{
	float dx, dy;
	pixelDiffFront(dx,dy);
	center(tag, cx, cy, dx, dy, Cglobal::instance().widthF, Cglobal::instance().heightF);
}

/** @brief handler für die Nachricht /ardrone/navdata
 *
 * speichert die Navdata-Informationen in den Variablen von Cglobal
 */
void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata)
{
	Cglobal::instance().altd = navdata->altd;
	Cglobal::instance().vx = navdata->vx;
	Cglobal::instance().vy = navdata->vy;
	Cglobal::instance().vz = navdata->vz;
	float x = navdata->rotX;
	float y = navdata->rotY;
	Cglobal::instance().roty = grad2rad(y);  //grad in rad umrechnen
	Cglobal::instance().rotx = grad2rad(x);

	gettimeofday(&Cglobal::instance().sinceNoNavdataUpdate, NULL);
}

/** @brief PD-Regelung für bottom_follow_tag
 */
void bottom_regulation()
{
	// P-Anteil:
	float ex = Cglobal::instance().twist.linear.x - Cglobal::instance().b_mmPs2twistx * Cglobal::instance().vx; //Fehler in x Richtung

	Cglobal::instance().twist.linear.x += Cglobal::instance().b_Kpx * ex;

	float ey = Cglobal::instance().twist.linear.y - Cglobal::instance().b_mmPs2twisty * Cglobal::instance().vy; //Fehler in y Richtung

	Cglobal::instance().twist.linear.y += Cglobal::instance().b_Kpy * ey;

	// D-Anteil:
	Cglobal::instance().twist.linear.x += Cglobal::instance().b_Kdx * (ex - Cglobal::instance().exold) / Cglobal::instance().Ta;
	Cglobal::instance().twist.linear.y += Cglobal::instance().b_Kdy * (ey - Cglobal::instance().eyold) / Cglobal::instance().Ta;

	Cglobal::instance().exold = ex;
	Cglobal::instance().eyold = ey;
}
/** @brief PD-Regelung für front_follow_tag
 */
void front_regulation()
{
	// P-Anteil:
	float ex = Cglobal::instance().twist.linear.x - Cglobal::instance().f_mmPs2twistx * Cglobal::instance().vx; //Fehler in x Richtung

	Cglobal::instance().twist.linear.x += Cglobal::instance().f_Kpx * ex;

	float ey = Cglobal::instance().twist.linear.y - Cglobal::instance().f_mmPs2twisty * Cglobal::instance().vy; //Fehler in y Richtung

	Cglobal::instance().twist.linear.y += Cglobal::instance().f_Kpy * ey;
// D-Anteil:
	Cglobal::instance().twist.linear.x += Cglobal::instance().f_Kdx * (ex - Cglobal::instance().exold) / Cglobal::instance().Ta;
	Cglobal::instance().twist.linear.y += Cglobal::instance().f_Kdy * (ey - Cglobal::instance().eyold) / Cglobal::instance().Ta;

	Cglobal::instance().exold = ex;
	Cglobal::instance().eyold = ey;
}
/** @brief P-Regelung für follow_tag
 */
void line_regulation()
{
	// P-Anteil:
	float ex = Cglobal::instance().twist.linear.x - Cglobal::instance().l_mmPs2twistx * Cglobal::instance().vx; //Fehler in x Richtung

	Cglobal::instance().twist.linear.x += Cglobal::instance().l_Kpx * ex;

	float ey = Cglobal::instance().twist.linear.y - Cglobal::instance().l_mmPs2twisty * Cglobal::instance().vy; //Fehler in y Richtung

	Cglobal::instance().twist.linear.y += Cglobal::instance().l_Kpy * ey;


}


} //namespace Math

namespace
{
	void center(const ar_recog::Tag& tag, float& cx, float& cy, float dx, float dy, float width, float height)
	{
		cx = 0;
		cy = 0;
		for(int i = 0; i < 7; i+=2)
		{
			cx = cx + tag.cwCorners[i];
			cy = cy + tag.cwCorners[i+1];
		}
		cy = cy / 4.0;
		cy += dy;
		cy = cy / height;

		cx = cx / 4.0;
		cx -= dx;
		cx = cx / width;
	}
	float grad2rad(float a)
	{
		return a *= 0.017453f;  // pi/180°
	}
}



