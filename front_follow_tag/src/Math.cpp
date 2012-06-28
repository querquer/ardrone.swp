#include "Math.h"
#include "Global.h"

/* @brief Berechnung der Anzahl der Pixel, um die das Tag aufgrund der Rotation verschoben wurde
 *
 * Problem: Sichtfeld ist noch nicht korrekt berechnet(derzeit für x und y Richtung gleich)
 *
 */
void Cmath::pixelDiff(float& x, float& y)
{

	  /*float ds = altd * tan( roty );  //Entfernung eingentliche position vom Tag (y) zur ermittelten
	  float c = (2 * altd) / tan(58); //Sichtfeldgröße
	  y = (ds * height) / c;      //Punkte des Tags werden um diese anzahl von Pixeln verschoben

	  float dsx = altd * tan( rotx );
	  x = (dsx * width) / c;

	  x /= 3;
	  y *= 0.5;

	  	  ostr << "roty     " << roty << endl;
	  ostr << "Delta s: " << ds << endl;
	  ostr << "c        " << c << endl;
	  ostr << "dy       " << dy << endl;
	  ostr << "delt s:  " << dsx << endl;
	  ostr << "dx       " << dx << endl;
	  */

	//y = 0.5 * tan(Cglobal::instance().roty) * 1.6 * Cglobal::instance().height;  //tan(58)~= 1.6
	//x = 0.5 * tan(Cglobal::instance().rotx) * 1.6 * Cglobal::instance().width;
	y = 0;
	x = 0;
}

/* @brief Berechnung aus den Eckpunkten wo sich das Tag bezüglich des Bildes befindet

	cy: 0 Tag ist am oberen Bildrand, 0.5 Tag ist in der Mitte(Höhe), 1 Tag ist am unteren Bildrand
	cx: 0 Tag ist am linken Bildrand, 0.5 Tag ist in der Mitte(Breite), 1 Tag ist am rechten Bildrend
*/
void Cmath::center(const ar_recog::Tag& tag, float& cx, float& cy)
{
	cx = 0;
	cy = 0;
	for(int i = 0; i < 7; i+=2)
	{
		cx = cx + tag.cwCorners[i];
		cy = cy + tag.cwCorners[i+1];
	}

	//Berechne um wie viele Pixel das Bild durch die Rotation verschoben wurde
	float dx, dy;
	pixelDiff(dx,dy);

	cy = cy / 4.0;
	cy += dy;
	cy = cy / Cglobal::instance().height;

	cx = cx / 4.0;
	//cx -= dx;
	cx += dx;
	cx = cx / Cglobal::instance().width;
}
