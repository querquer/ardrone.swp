#include "std_includes.h"

using namespace std;

/**
 * @file front_follow_tag.cpp
 * @brief Applikation zur Tagvervolgung mit der vorderen Kamera
 */

void handleTag(const ar_recog::Tags::ConstPtr& msg) {
	ostringstream ostr;
	if (msg->tag_count == 0) //Falls kein Tag erkannt wurde
			{
		if (Cglobal::instance().seen) //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
			Cglobal::instance().sinceNotSeen = time(NULL);
		//if(time(NULL) - Cglobal::instance().sinceNotSeen < 1)  //Falls ein Tag in der letzen Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
		if (time(NULL) - Cglobal::instance().sinceNotSeen < 3) //n.n.t
		{
			Cglobal::instance().twist = Cglobal::instance().twist_old;
		} else //Versuche ruhig in Luft zu stehen
		{
			Cglobal::instance().twist.linear.x = 0;
			Cglobal::instance().twist.linear.z = 0;
			Cglobal::instance().twist.linear.y = 0;

			Cglobal::instance().twist.angular.z = 0;
			//if (time(NULL) - Cglobal::instance().sinceNotSeen > 0.5) //Falls länger als 0.5s kein Tag zu sehen war, drehe langsamer
				//Cglobal::instance().twist.angular.z = 0.5
					//	* Cglobal::instance().lastDir;
		}
		Cglobal::instance().seen = false;
	} else //Falls mindestens ein Tag gesehen wurde
	{
		Cglobal::instance().seen = true;
		ar_recog::Tag biggest = msg->tags[0];

		for (int i = 0; i < msg->tag_count; ++i) {
			if (msg->tags[i].diameter > biggest.diameter)
				biggest = msg->tags[i];
		}

		float cx = 0;
		float cy = 0;
		Math::centerFront(biggest, cx, cy);

		ostr << "cy      :" << cy << endl;
		ostr << "cx     :" << cx << endl;
		ostr << "dist    :" << biggest.distance << endl;
		ostr << "Tag: yRot: " << biggest.yRot << endl;
		if(biggest.yRot < 0)
			ostr << "-----------------------------------" << endl << "----------------------------" << endl;

		float stopping_dist = 1700.0;

		float dist = (biggest.distance - stopping_dist) / stopping_dist;


		float dist_vel = Cglobal::instance().lxDelta.get_velocity(dist);

		if (abs(dist) < 0.25)
			Cglobal::instance().twist.linear.x = dist_vel * 0.2; //if we are close enough to the stopping distance, just try to stop
		else
			Cglobal::instance().twist.linear.x = dist * 0.25; //otherwise try to move within stopping_dist

		if(abs(biggest.distance - stopping_dist) < 200)
			Cglobal::instance().twist.linear.x = 0;

		Cglobal::instance().twist.linear.x = max(-0.08, min(0.08, Cglobal::instance().twist.linear.x));

		//try to face perpendicular to the tag
		//float yRot_velocity = Cglobal::instance().azDelta.get_velocity(biggest.yRot);
		//Cglobal::instance().twist.angular.z = -(yRot_velocity * 0.6);
		if(biggest.yRot < 0)
		{
			Cglobal::instance().twist.angular.z = 0.3;
		}
		else
		{
			Cglobal::instance().twist.angular.z = -0.3;
		}
		if(abs(biggest.yRot) < 0.4)
			Cglobal::instance().twist.angular.z = 0;

		Cglobal::instance().twist.angular.z = max(-0.5, min(0.5, Cglobal::instance().twist.angular.z));
		//if (abs(biggest.yRot) < 0.2)
		//twist.linear.y = yRot_velocity * 0.25; //if we are mostly facing perpendicular, just try to stay still
		//else
		//twist.linear.y = biggest.yRot * 0.125; //otherwise, rotate towards being in front of the tag

		//rotate to face the tag
		Cglobal::instance().twist.linear.y = (-(cx - 0.5) / 0.5) * 0.5;
		//Cglobal::instance().twist.linear.y = Cglobal::instance().lyDelta.get_velocity(Cglobal::instance().twist.linear.y);
		Cglobal::instance().twist.linear.y = max(-0.08, min(0.08, Cglobal::instance().twist.linear.y));


		if(abs(biggest.yRot) > 0.4)
		{
			Cglobal::instance().twist.linear.y *= 1.5f;
		}
		if((cx < 0.3 && Cglobal::instance().twist.angular.z < 0)
				|| (cx > 0.7 && Cglobal::instance().twist.angular.z > 0))
		{
			Cglobal::instance().twist.angular.z = 0;
		}
		/*if(abs(biggest.yRot) > 0.4)
		{
			Cglobal::instance().twist.linear.y = 0;
		}
		else
		{
			Cglobal::instance().twist.angular.z = 0;
		}*/


		if (Cglobal::instance().twist.angular.z < 0)
			Cglobal::instance().lastDir = -1;
		else
			Cglobal::instance().lastDir = 1;

		Cglobal::instance().twist.linear.z = (-(cy - 0.5) / 0.5);
		/*if (Cglobal::instance().twist.linear.z > 0.5)
			Cglobal::instance().twist.linear.z = 0.1;
		else if (Cglobal::instance().twist.linear.z < -0.4)
			Cglobal::instance().twist.linear.z = -0.1;
		else
			Cglobal::instance().twist.linear.z = 0;*/
		if(cy > 0.3 && cy < 0.7)
			Cglobal::instance().twist.linear.z = 0;
		//Cglobal::instance().twist.linear.z = max(-0.1,	min(0.3, Cglobal::instance().twist.linear.z));


	}


	if ((Cglobal::instance().altd > 1700 && Cglobal::instance().twist.linear.z > 0) || (Cglobal::instance().altd < 500 && Cglobal::instance().twist.linear.z < 0))
		Cglobal::instance().twist.linear.z = 0;

	Cglobal::instance().twist_old = Cglobal::instance().twist;


	ostr << "vx   " << Cglobal::instance().vx << endl;
	ostr << "vy   " << Cglobal::instance().vy << endl;
	ostr << "t.l.x   " << Cglobal::instance().twist.linear.x << endl;
	ostr << "t.l.y   " << Cglobal::instance().twist.linear.y << endl;
	ostr << "roty: " << Cglobal::instance().roty << endl;

	/*
	 * hier PID-Regler:
	 * -Sollwert: twist.linear
	 * -Istwert: vx,vy,vz
	 *
	 * twist wert = 1 -> vx soll 5000 sein
	 *
	 */
	Math::front_regulation();
/*
// P-Anteil:
	float mmPs2twistx = 0.0002f; //weil Drone in  x Richtung max 5m/s fliegt
	float mmPs2twisty = 0.0002f;

	float ex = Cglobal::instance().twist.linear.x - mmPs2twistx * Cglobal::instance().vx; //Fehler in x Richtung

	float Kpx = 1.5f;

	Cglobal::instance().twist.linear.x += Kpx * ex;

	float ey = Cglobal::instance().twist.linear.y - mmPs2twisty * Cglobal::instance().vy; //Fehler in y Richtung

	float Kpy = 2.5f;

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

	ostr << "linear.x: " << Cglobal::instance().twist.linear.x << endl;
	ostr << "linear.y: " << Cglobal::instance().twist.linear.y << endl;
	ostr << "linear.z: " << Cglobal::instance().twist.linear.z << endl;
	ostr << "angular.z: " << Cglobal::instance().twist.angular.z << endl
			<< endl;
	if (msg->tag_count > 0)
		ostr << "Erkannt!!\nDistance: " << msg->tags[0].distance << endl;

	ostr << endl << "-----------------------------------------------------------------" << endl;
	ROS_INFO(ostr.str().c_str());

	Cglobal::instance().of << ostr.str();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "follow_tag");

	ros::NodeHandle node_handle;
	Cglobal::instance().pub = node_handle.advertise < geometry_msgs::Twist
			> ("cmd_vel", 1000);
	ros::Subscriber sub = node_handle.subscribe("tags", 1000, handleTag);

	ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000,
			Math::navdataUpdate);

	while (!Cglobal::instance().end && ros::ok()) {
		ros::spinOnce();
	}

	Cglobal::destroy();
}
