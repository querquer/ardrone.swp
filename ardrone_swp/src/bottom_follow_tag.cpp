#include "std_includes.h"

using namespace std;

/**
 * @file bottom_follow_tag.cpp
 * @brief Applikation zur Tagvervolgung mit der unteren Kamera
 */

/** @brief subscriber handler für die Nachricht tags
 */
void handleTag(const ar_recog::Tags::ConstPtr& msg)
{
	ostringstream ostr;
	if (msg->tag_count == 0) //Falls kein Tag erkannt wurde
	{
		Cglobal::instance().exsum = 0;
		Cglobal::instance().eysum = 0;

		if (Cglobal::instance().seen) //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
			Cglobal::instance().sinceNotSeen = time(NULL);
		//if(time(NULL) - Cglobal::instance().sinceNotSeen < 1)  //Falls ein Tag in der letzen Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
		if (time(NULL) - Cglobal::instance().sinceNotSeen < 3) //n.n.t
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
		{
			if (msg->tags[i].diameter > biggest.diameter)
				biggest = msg->tags[i];
		}

		float cx = 0;
		float cy = 0;
		/*
		 * Bestimme die Mittelpunkte
		 */
		Math::centerBottom(biggest, cx, cy);

		ostr << "cy      :" << cy << endl;
		ostr << "cx     :" << cx << endl;

		/*
		 etwa bei einer Höhe von 1,3 m schweben
		 */
		if (biggest.distance > 1350.0f)
		{
			Cglobal::instance().twist.linear.z = -0.1;
		}
		else if (biggest.distance < 1200.0f)
			Cglobal::instance().twist.linear.z = 0.3;
		else
			Cglobal::instance().twist.linear.z = 0;

		/*
		 * Geschwindigkeit in x un y Richtung steigt linear mit der Entfernung zum Mittelpunkt
		 * 0.08 ist das Maximum
		 */
		Cglobal::instance().twist.linear.y = max(-0.08, min(0.08, -(cx - 0.5) / 4));
		Cglobal::instance().twist.linear.x = max(-0.08, min(0.08, -(cy - 0.5) / 4));

		/*float rotZ = biggest.zRot;    //versuche längs zum Tag zu sein, dabei entweder vorne oder hinten(was näher dran ist)
		 if(rotZ > 1.5)
		 rotZ = rotZ - 3;
		 if(rotZ < -1.5)
		 rotZ = rotZ + 3;*/

		//twist.angular.z = azDelta.get_velocity(-biggest.zRot * ( 0.66667 ));  // Drehe dich in Abhängigkeit von der z Rotation des Tags
		Cglobal::instance().twist.angular.z = -(max(-1.5, min(1.5, biggest.zRot)) * (0.666667));
	}

	struct timeval timenow;
	gettimeofday(&timenow, NULL);
	if (timenow.tv_sec - Cglobal::instance().sinceNoNavdataUpdate.tv_sec > 1
			|| (timenow.tv_sec - Cglobal::instance().sinceNoNavdataUpdate.tv_sec == 0 && timenow.tv_usec - Cglobal::instance().sinceNoNavdataUpdate.tv_usec > 500000))
	{
		ostr << "keine neuen navdata" << endl;
		Cglobal::instance().twist.linear.x = 0;
		Cglobal::instance().twist.linear.z = 0;
		Cglobal::instance().twist.linear.y = 0;
		Cglobal::instance().twist.angular.z = 0;
	}

	//Fliege nicht höher als 1,7m und nicht niedriger als 0,3m
	if ((Cglobal::instance().altd > 1700
			&& Cglobal::instance().twist.linear.z > 0)
			|| (Cglobal::instance().altd < 300
					&& Cglobal::instance().twist.linear.z < 0))
		Cglobal::instance().twist.linear.z = 0;

	Cglobal::instance().twist_old = Cglobal::instance().twist;

	ostr << "vx   " << Cglobal::instance().vx << endl;
	ostr << "vy   " << Cglobal::instance().vy << endl;
	ostr << "t.l.x   " << Cglobal::instance().twist.linear.x << endl;
	ostr << "t.l.y   " << Cglobal::instance().twist.linear.y << endl;


	/*
	 * hier PID-Regler:
	 * -Sollwert: twist.linear
	 * -Istwert: vx,vy,vz
	 *
	 * twist wert = 1 -> vx soll 5000 sein
	 *
	 */
	Math::bottom_regulation();
/*
// P-Anteil:
	float mmPs2twistx = 0.0002f; //weil Drone in  x Richtung max 5m/s fliegt
	float mmPs2twisty = 0.0002f;

	float ex = Cglobal::instance().twist.linear.x - mmPs2twistx * Cglobal::instance().vx; //Fehler in x Richtung

	float Kpx = 2.5f;

	Cglobal::instance().twist.linear.x += Kpx * ex;

	float ey = Cglobal::instance().twist.linear.y - mmPs2twisty * Cglobal::instance().vy; //Fehler in y Richtung

	float Kpy = 1.5f;

	Cglobal::instance().twist.linear.y += Kpy * ey;

	ostr << "P: x:  " << Kpx * ex << endl;
	ostr << "P: y:  " << Kpy * ey << endl;

	float Ta = 0.05555555;
// D-Anteil:
	float Kdx = 0.05f; //Diese werte müssen noch angepasst werden und kommen dann noch in Cglobal als static Variable rein
	float Kdy = 0.05f;
	Cglobal::instance().twist.linear.x += Kdx * (ex - Cglobal::instance().exold) / Ta;
	Cglobal::instance().twist.linear.y += Kdy * (ey - Cglobal::instance().eyold) / Ta;

	ostr << "D: x:  " << Kdx * (ex - Cglobal::instance().exold) / Ta << endl;
	ostr << "D: y:  " << Kdy * (ey - Cglobal::instance().eyold) / Ta << endl;

	Cglobal::instance().exold = ex;
	Cglobal::instance().eyold = ey;

*/

	Cglobal::instance().pub.publish(Cglobal::instance().twist);

	ostr << "\nrotx:   " << Cglobal::instance().rotx << endl;
	ostr << "\nroty:   " << Cglobal::instance().roty << endl;
	ostr << "\n\nlinear.x: " << Cglobal::instance().twist.linear.x << endl;
	ostr << "linear.y: " << Cglobal::instance().twist.linear.y << endl;
	ostr << "linear.z " << Cglobal::instance().twist.linear.z << endl;
	ostr << "angular.z " << Cglobal::instance().twist.angular.z << endl;
	ostr << "altd: " << Cglobal::instance().altd << endl;
	if (msg->tag_count > 0)
		ostr << "Distance: " << msg->tags[0].distance << endl;
	ostr << endl << "-----------------------------------------------------------------" << endl;

	ROS_INFO(ostr.str().c_str());
	Cglobal::instance().of << ostr.str();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow_tag_bottom");

	ros::NodeHandle node_handle;
	Cglobal::instance().pub = node_handle.advertise < geometry_msgs::Twist > ("cmd_vel", 1000);
	ros::Subscriber sub = node_handle.subscribe("tags", 1000, handleTag);

	ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000,
			Math::navdataUpdate);

	while (!Cglobal::instance().end && ros::ok())
	{
		ros::spinOnce();
	}
	Cglobal::destroy();
}
