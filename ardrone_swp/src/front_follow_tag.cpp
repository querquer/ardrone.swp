/**
 * @file front_follow_tag.cpp
 * @brief Applikation zur Tagvervolgung mit der vorderen Kamera
 *
 * Die Applikation published eine Message vom Typ cmd_vel(Twist-Objekt) zum steuern der Drone immer dann,\n
 * wenn eine neue Message vom Typ tags von der Applikation ar_recog gesendet wurde. Dies geschieht etwa 18 mal pro Sekunde.
 * \n\n
 * Die Drone hält zu einem Tag einen Abstand von etwa 1,5m. \n
 * Falls das Tag am oberen oder unteren Bildrand gesehen wird, passt die Drone ihre Höhe an.\n
 * In y-Richtung fliegt die Drone immer so, sodass das Tag in der Bildmitte ist.\n
 * Ist das Tag mehr als 24° seitlich verdreht, dreht sich die Drone, sodass sie frontal zum Tag steht.\n
 * Wurde kein Tag erkannt, fliegt die Drone drei Sekunden in die Richtung, in der sie das Tag zuletzt gesehen hat.\n
 * Hat die Drone das Tag nach 3 Sekunden noch nicht wiedergefunden, fliegt sie auf der Stelle.\n
 * \n
 * Die Regelung erfolgt in Math::front_regulation() mit Hilfe eines PD-Reglers.\n
 */
#include "std_includes.h"

using namespace std;


/** @brief handler für die Nachricht tags: hier werden die Bewegungsdaten gesetzt und gepublished
 *
 */
void handleTag(const ar_recog::Tags::ConstPtr& msg) {
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

		for (int i = 0; i < msg->tag_count; ++i)   //Suche des größte Tag
			if (msg->tags[i].diameter > biggest.diameter)
				biggest = msg->tags[i];

		float cx = 0;
		float cy = 0;
		Math::centerFront(biggest, cx, cy);  //ermittle den Mittelpunkt

		float stopping_dist = 1700.0; //Distanz bei der die Drohne stoppen soll
		float dist = (biggest.distance - stopping_dist) / stopping_dist;

		if (abs(dist) < 0.25)
			Cglobal::instance().twist.linear.x = dist * 0.2; //Wenn die Drone nahe am Tag ist, soll sie langsam fliegen
		else
			Cglobal::instance().twist.linear.x = dist * 0.25; //sonst schneller

		//wenn die Distanz zum Tag näher als 200 an der Stoppdistenz ist, soll sie sich in x-Ricktung nicht bewegen,
		//damit sie in x Richtung nicht immer in Bewegung ist
		if(abs(biggest.distance - stopping_dist) < 200)
			Cglobal::instance().twist.linear.x = 0;

		Cglobal::instance().twist.linear.x = max(-0.08, min(0.08, Cglobal::instance().twist.linear.x));

		//richte dich zum tag aus, aber nur wenn der winkel groß ist
		if(biggest.yRot < 0)
			Cglobal::instance().twist.angular.z = 0.3;
		else
			Cglobal::instance().twist.angular.z = -0.3;
		if(abs(biggest.yRot) < 0.4)
			Cglobal::instance().twist.angular.z = 0;

		//bewege dich seitwätrs, sodass das Tag in der Mitte des Bildes ist
		//Maximum: 0,08
		Cglobal::instance().twist.linear.y = max(-0.08, min(0.08, (-(cx - 0.5) / 0.5) * 0.5));
		//falls du dich zusätzlich drehst, bewege dich um 50% schneller in y-Richtung
		if(abs(biggest.yRot) > 0.4)
			Cglobal::instance().twist.linear.y *= 1.5f;
		//Falls das Tag am Bildrand ist und durch eine Drehung das Tag noch weiter außen zu sehen wäre: drehe dich nicht
		if((cx < 0.3 && Cglobal::instance().twist.angular.z < 0)
				|| (cx > 0.7 && Cglobal::instance().twist.angular.z > 0))
			Cglobal::instance().twist.angular.z = 0;

		//Sezte Geschwindigkeit in z-Richtung
		Cglobal::instance().twist.linear.z = (-(cy - 0.5) / 0.5);
		//Bewege dich nur nach unten oder oben, wenn das Tag am Bildrand ist
		if(cy > 0.3 && cy < 0.7)
			Cglobal::instance().twist.linear.z = 0;
	}

	//Fliege nicht höher als 1,7m und nicht niedriger als 0,5m
	if ((Cglobal::instance().altd > 1700 && Cglobal::instance().twist.linear.z > 0) || (Cglobal::instance().altd < 500 && Cglobal::instance().twist.linear.z < 0))
		Cglobal::instance().twist.linear.z = 0;

	//Speichere die Bewegungsdaten, falls beim nächsten Mal kein Tag erkannt wird
	Cglobal::instance().twist_old = Cglobal::instance().twist;

	Math::front_regulation(); //Regelung

	//Keyboard::control(); Falls es möglich sein soll mit der Tastatur zu steuern, Kommentar wegnehmen

	Cglobal::instance().pub.publish(Cglobal::instance().twist); //Bewegungsdaten publishen
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "follow_tag");

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
