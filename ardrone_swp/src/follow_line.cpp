/**
 * @file follow_line.cpp
 * @brief Applikation zur Linienverfolgung mit der unteren Kamera
 *
 * Die Applikation published eine Message vom Typ cmd_vel(Twist-Objekt) zum steuern der Drone immer dann,\n
 * wenn eine neue Message vom Typ LinePos von der Applikation TrackLine gesendet wurde. \n
 * Die Message LinePos enthält 3 float Werte:
 * - x: gibt an wo sich die Linie befindet(links, rechts). Mitte: 80
 * - y: bleibt immer bei 60(Mitte)
 * - angle: gibt den Winkel der Linie bezüglich der Drone im Bereich [-90°, 90°] an. Die Werte von angle liegen zwischen -9000 und 9000\n\n
 *
 * Wenn das erste Mal eine Linie erkannt wird, fliegt die Drone auf eine Höhe von etwa 0,9m. Danach beginnt sie der Linie auf derselben Höhe zu folgen.\n\n
 *
 * Falls die Linie einen Winkel von weniger als 9° bezüglich der Drone hat, führt sie 3 Bewegungen aus:\n
 * - langsam drehen, sodass der Winkel 0° groß wird \n
 * - nach vorne fliegen \n
 * - nach links bzw. rechts fliegen, sodass die Linie in der Mitte des Bildes ist \n
 * \n
 *
 * Falls der Winkel größer als 9° ist, versucht die Drone erst ruhig auf der Stelle zu fliegen.\n
 * Wenn sich die Drone kaum noch bewegt, dreht sie sich, sodass der Winkel kleiner wird.\n
 *
 * Die Drone soll sich nicht über einen längeren Zeitraum drehen und gleichzeitig in x- oder y-Richtung fliegen, \n
 * da sie sonst sehr schnell und unkontrolliert fliegt.\n
 * Zudem weicht bei einem großen Winkel der x-Wert von TrackLine sehr weit von der Mitte ab, auch wenn die Drone weiterhin über der Linie steht.\n
 * Deshalb kann bei einem großen Winkel auch nicht in y-Richtung geflogen werden. \n\n
 *
 * Die Applikation funktioniert nicht bei lang gestreckten Kurven, da eine gleichzeitige Dreh-Bewegung und x-oder y-Bewegung zu unkontrollierten, schnellen Bewegungen führt.\n
 * Statt Kurven sollte die Linie "Knicke" haben(siehe Video).\n\n
 *
 * Die Regelung erfolgt in Math::line_regulation() mit Hilfe eines P-Reglers.
 */

/**
 * @file TrackLine.py
 * @brief Applikation zur Liniendedektion
 *
 * published Messages vom Typ LinePos: \n
 * Die Message LinePos enthält 3 float Werte:
 * - x: gibt an wo sich die Linie befindet(links, rechts). Mitte: 80
 * - y: bleibt immer bei 60(Mitte)
 * - angle: gibt den Winkel der Linie bezüglich der Drone im Bereich [-90°, 90°] an. Die Werte von angle liegen zwischen -9000 und 9000\n\n
 */
#include "std_includes.h"
#include "ardrone_swp/LinePos.h"

using namespace std;

/** @brief handler für die Nachricht LinePos: hier werden die Bewegungsdaten gesetzt und gepublished
 *
 */
void handleLine(const ardrone_swp::LinePos::ConstPtr& msg)
{
	ostringstream ostr;
	if (msg->x == 80 && msg->y == 60)  //Falls keine Linie erkannt wird, versuche auf der Stelle zu bleiben
	{
		Cglobal::instance().twist.linear.x = 0;
		Cglobal::instance().twist.linear.z = 0;
		Cglobal::instance().twist.linear.y = 0;
		Cglobal::instance().twist.angular.z = 0;
	}
	else
	{

		float angle = 900.0f;  //ab diesem Winkel nur drehen

		//ein Winkel < 900: nach vorne fliegen, y und Dreh-Geschwindigkeit anpassen
		if(abs(msg->angle) < angle)
		{
			Cglobal::instance().twist.linear.x = 0.02;
			Cglobal::instance().twist.angular.z = -(msg->angle / 4000.0f);

			//falls der Winkel zu hoch ist, ist der msg->x Wert zu weit von 80 entfernt
			Cglobal::instance().twist.linear.y = -(msg->x - 80.0f) / (80.0f * 10.0f);
		}
		else //Wenn es einen Winkel > 900 gibt: nur drehen, denn beim Drehen werden die Geschindigkeiten in x und y Richtung zu groß
		{
			Cglobal::instance().twist.linear.x = 0;
			Cglobal::instance().twist.linear.y = 0;

			if(msg->angle > 0)
				Cglobal::instance().twist.angular.z = -0.6;
			else
				Cglobal::instance().twist.angular.z = 0.6;
			//bevor gedreht wird, wird so lange gewartet bis die Drohne sich kaum noch in x und y Richtung bewegt
			if(abs(Cglobal::instance().vx) > 150 || abs(Cglobal::instance().vy) > 150)
				Cglobal::instance().twist.angular.z = 0;
		}
	}
	//fliege auf einer Höhe von etwa 0.9m bis 1m
	if (Cglobal::instance().altd > 1050)
		Cglobal::instance().twist.linear.z = -0.1;
	else if (Cglobal::instance().altd < 900)
		Cglobal::instance().twist.linear.z = 0.3;
	else
		Cglobal::instance().twist.linear.z = 0;

	//Beim Start: fliegt erst einmal über 0.9m hoch, dann folge der Linie
	if(Cglobal::instance().altd < 900 && Cglobal::instance().begin)
	{
		Cglobal::instance().twist.linear.x = 0;
		Cglobal::instance().twist.linear.y = 0;
		Cglobal::instance().twist.angular.z = 0;
		Cglobal::instance().twist.linear.z = 0.5;
	}
	else
		Cglobal::instance().begin = false;

	//Fliege nicht höher als 1,7m und nicht niedriger als 0,3m
	if ((Cglobal::instance().altd > 1700 && Cglobal::instance().twist.linear.z > 0)
			|| (Cglobal::instance().altd < 300 && Cglobal::instance().twist.linear.z < 0))
		Cglobal::instance().twist.linear.z = 0;


	Math::line_regulation(); //Regelung

	//Keyboard::control(); Falls es möglich sein soll mit der Tastatur zu steuern, Kommentar wegnehmen

	Cglobal::instance().pub.publish(Cglobal::instance().twist); //Bewegungsdaten publishen
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "follow_line");

	ros::NodeHandle node_handle;
	Cglobal::instance().pub = node_handle.advertise < geometry_msgs::Twist > ("cmd_vel", 1000);
	ros::Subscriber sub = node_handle.subscribe("LinePos", 1000, handleLine);

	ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, Math::navdataUpdate);

	while (!Cglobal::instance().end && ros::ok())
	{
		ros::spinOnce();
	}

	Cglobal::destroy();
}
