/**
 * @file follow_line.cpp
 * @brief Applikation zur Linienverfolgung mit der unteren Kamera
 *
 * TODO : Ausführliche Beschreibung
 */
#include "std_includes.h"
#include "ardrone_swp/LinePos.h"

using namespace std;

/** @brief handler für die Nachricht LinePos
 * hier werden die Bewegungsdaten gesetzt und gepublished
 *
 * TODO : Ausführliche Beschreibung
 */
void handleLine(const ardrone_swp::LinePos::ConstPtr& msg)
{
	ostringstream ostr;
	if (msg->x == 80 && msg->y == 60)
	{
		if (Cglobal::instance().seen) //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
			Cglobal::instance().sinceNotSeen = time(NULL);
		//Falls ein Tag in den letzen  2 Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
		if (time(NULL) - Cglobal::instance().sinceNotSeen < 2)
		{
			Cglobal::instance().twist = Cglobal::instance().twist_old;
		}
		else //Versuche ruhig in Luft zu stehen
		{
			Cglobal::instance().twist.linear.x = 0;
			Cglobal::instance().twist.linear.z = 0;
			Cglobal::instance().twist.linear.y = 0;
			Cglobal::instance().twist.angular.z = 0;
		}
		Cglobal::instance().seen = false;
	}
	else
	{
		Cglobal::instance().seen = true;

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


	Cglobal::instance().twist_old = Cglobal::instance().twist; //Speichere die Bewegungsdaten, falls beim nächsten Mal kein Tag erkannt wird

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
