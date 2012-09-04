/**
 * @file bottom_follow_tag.cpp
 * @brief Applikation zur Tagvervolgung mit der unteren Kamera
 *
 * Die Applikation published eine Message vom Typ cmd_vel(Twist-Objekt) zum steuern der Drone immer dann,\n
 * wenn eine neue Message vom Typ tags von der Applikation ar_recog gesendet wurde. Dies geschieht etwa 18 mal pro Sekunde.
 * \n\n
 * Wurde ein Tag erkannt, soll die Drone darauf zu fliegen, sodass das Tag in den Mittelpunkt des Bildes kommt. \n
 * Dabei fliegt die Drone bei einer Höhe von etwa 1,3m über dem Tag.\n
 * Wurde kein Tag erkannt, fliegt die Drone drei Sekunden in die Richtung, in der sie das Tag zuletzt gesehen hat.\n
 * Hat die Drone das Tag nach 3 Sekunden noch nicht wiedergefunden, fliegt sie auf der Stelle.\n
 * \n
 * Die Regelung erfolgt in Math::bottom_regulation() mit Hilfe eines PD-Reglers.\n
 * Sind über eine halbe Sekunde lang keine neuen Navigationsdaten(Message ardrone/navdata) von der Drone angekommen(passiert teilweise bei gestörter WLAN-Verbindung), \n
 * bleibt die Drohne stehen. Sonst würde die Drone aufgrund der Regelung mit den alten Geschwindigkeitswerten nicht das gewünschte Verhalten zeigen.
 */

#include "std_includes.h"

using namespace std;


/** @brief handler für die Nachricht tags: hier werden die Bewegungsdaten gesetzt und gepublished
 *

 */
void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
	if (msg->tag_count == 0) //Falls kein Tag erkannt wurde
	{
		if (Cglobal::instance().seen) //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
			Cglobal::instance().sinceNotSeen = time(NULL);
		//Falls ein Tag in den letzen 3 Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
		if (time(NULL) - Cglobal::instance().sinceNotSeen < 3)
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
	else //Falls mindestens ein Tag gesehen wurde
	{
		Cglobal::instance().seen = true;
		ar_recog::Tag biggest = msg->tags[0];

		for (unsigned int i = 0; i < msg->tag_count; ++i) //Suche das größte Tag
			if (msg->tags[i].diameter > biggest.diameter)
				biggest = msg->tags[i];

		float cx = 0;
		float cy = 0;
		Math::centerBottom(biggest, cx, cy); //Bestimme die Mittelpunkte

		// etwa bei einer Höhe von 1,3 m schweben
		if (biggest.distance > 1350.0f)
		{
			Cglobal::instance().twist.linear.z = -0.1;
		}
		else if (biggest.distance < 1200.0f)
			Cglobal::instance().twist.linear.z = 0.3;
		else
			Cglobal::instance().twist.linear.z = 0;

		/* Geschwindigkeit in x und y Richtung steigt linear mit der Entfernung zum Mittelpunkt
		 * 0.08 ist das Maximum
		 * (cx - 0.5), weil 0.5 die Mitte ist
		 * durch 4 wird geteilt, um die Bewegung langsamer zu machen(dieser Wert wurde ausprobiert, mit 4 wurden die besten Ergebnisse erziehlt)
		 */
		Cglobal::instance().twist.linear.y = max(-0.08, min(0.08, -(cx - 0.5) / 4));
		Cglobal::instance().twist.linear.x = max(-0.08, min(0.08, -(cy - 0.5) / 4));

		/* Drehe dich in Abhängigkeit von der z Rotation des Tags
		 * ab 90°(zRot = 1.5): maximale Drehgeschwindigkeit
		 */
		Cglobal::instance().twist.angular.z = -(max(-1.5, min(1.5, biggest.zRot)) * (0.666667));
	}

	struct timeval timenow;
	gettimeofday(&timenow, NULL);
	if (timenow.tv_sec - Cglobal::instance().sinceNoNavdataUpdate.tv_sec > 1
			|| (timenow.tv_sec - Cglobal::instance().sinceNoNavdataUpdate.tv_sec == 0 && timenow.tv_usec - Cglobal::instance().sinceNoNavdataUpdate.tv_usec > 500000))
	{
		//Falls eine halbe Sekunde oder länger keine neuen Navigationsdaten gesendet wurden, versuche ruhig zu stehen
		Cglobal::instance().twist.linear.x = 0;
		Cglobal::instance().twist.linear.z = 0;
		Cglobal::instance().twist.linear.y = 0;
		Cglobal::instance().twist.angular.z = 0;
		//Speichere die Bewegungsdaten, falls beim nächsten Mal kein Tag erkannt wird
		Cglobal::instance().twist_old = Cglobal::instance().twist;
		Cglobal::instance().pub.publish(Cglobal::instance().twist);  //Bewegungsdaten publishen
		return;
	}

	//Fliege nicht höher als 1,7m und nicht niedriger als 0,3m
	if ((Cglobal::instance().altd > 1700 && Cglobal::instance().twist.linear.z > 0)
			|| (Cglobal::instance().altd < 300 && Cglobal::instance().twist.linear.z < 0))
		Cglobal::instance().twist.linear.z = 0;

	//Speichere die Bewegungsdaten, falls beim nächsten Mal kein Tag erkannt wird
	Cglobal::instance().twist_old = Cglobal::instance().twist;

	Math::bottom_regulation(); //Regelung

	//Keyboard::control(); Falls es möglich sein soll mit der Tastatur zu steuern, Kommentar wegnehmen

	Cglobal::instance().pub.publish(Cglobal::instance().twist);  //Bewegungsdaten publishen
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow_tag_bottom");

	ros::NodeHandle node_handle;
	Cglobal::instance().pub = node_handle.advertise < geometry_msgs::Twist > ("cmd_vel", 1000);
	ros::Subscriber sub = node_handle.subscribe("tags", 1000, handleTag);

	ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000, Math::navdataUpdate);

	while (!Cglobal::instance().end && ros::ok())
	{
		ros::spinOnce();
	}
	Cglobal::destroy();
}
