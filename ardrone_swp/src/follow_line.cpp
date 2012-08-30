#include "std_includes.h"
#include "ardrone_swp/LinePos.h"

using namespace std;

/**
 * @file follow_line.cpp
 * @brief Applikation zur Linienverfolgung mit der unteren Kamera
 */
bool begin = true;
struct timeval start;
struct timeval ac_time;
void handleLine(const ardrone_swp::LinePos::ConstPtr& msg)
{
	ostringstream ostr;
	if (msg->x == 80 && msg->y == 60)
	{
		if (Cglobal::instance().seen) //Falls beim letzen Aufruf ein Tag gesehen wurde, speichere die Zeit
			Cglobal::instance().sinceNotSeen = time(NULL);
		//if(time(NULL) - Cglobal::instance().sinceNotSeen < 1)  //Falls ein Tag in der letzen Sekunde gesehen wurde, versuche in die letzte gesehene Richtung zu fliegen
		if(false)
		//if (time(NULL) - Cglobal::instance().sinceNotSeen < 2) //n.n.t
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

		if(abs(msg->angle) < angle)
		{
			Cglobal::instance().twist.linear.x = 0.02;
			Cglobal::instance().twist.angular.z = -(msg->angle / 4000.0f);
			//Cglobal::instance().twist.angular.z = 0;
			Cglobal::instance().twist.linear.y = -(msg->x - 80.0f) / (80.0f * 10.0f); //falls der Winkel zu hoch ist, wird ein x wert ausgegeben, der zu sehr von 80 entfernt ist
/*
			//Cglobal::instance().twist.angular.z = 0;
			//Cglobal::instance().twist.linear.y = -(msg->x - 80.0f) / (80.0f * 9.0f);
			if (abs(msg->x - 80) < 30)
			{
				Cglobal::instance().twist.linear.x = 0.03;
				//Cglobal::instance().twist.angular.z = 0;
				Cglobal::instance().twist.angular.z = -(msg->angle / 4000.0f);
				Cglobal::instance().twist.linear.y = -(msg->x - 80.0f) / (80.0f * 9.0f); //falls der Winkel zu hoch ist, wird ein x wert ausgegeben, der zu sehr von 80 entfernt ist
			}
			else
			{
				Cglobal::instance().twist.linear.x = 0;
				//Cglobal::instance().twist.angular.z = 0;
				//Cglobal::instance().twist.linear.y = -(msg->x - 80.0f) / (80.0f * 9.0f);
			}
			*/
		}
		else
		{
			Cglobal::instance().twist.linear.x = 0;
			Cglobal::instance().twist.linear.y = 0;

			if(msg->angle > 0)
				Cglobal::instance().twist.angular.z = -0.6;
			else
				Cglobal::instance().twist.angular.z = 0.6;
			if(abs(Cglobal::instance().vx) > 150 || abs(Cglobal::instance().vy) > 150)
				Cglobal::instance().twist.angular.z = 0;
		}

		ostr << "x: " << msg->x << endl;
		ostr << "y: " << msg->y << endl;
		ostr << "angle: " << msg->angle << endl;
	}


	if (Cglobal::instance().altd > 1050)
		Cglobal::instance().twist.linear.z = -0.1;
	else if (Cglobal::instance().altd < 900)
		Cglobal::instance().twist.linear.z = 0.3;
	else
		Cglobal::instance().twist.linear.z = 0;

	if(Cglobal::instance().altd < 900 && begin)
	{
		Cglobal::instance().twist.linear.x = 0;
		Cglobal::instance().twist.linear.y = 0;
		Cglobal::instance().twist.angular.z = 0;
		Cglobal::instance().twist.linear.z = 0.5;
	}
	else
	{
		if(begin)
			gettimeofday(&start, NULL);
		begin = false;
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
	//Math::line_regulation();

// P-Anteil:
	float mmPs2twistx = 0.00015f; //weil Drone in  x Richtung max 5m/s fliegt
	float mmPs2twisty = 0.00025f;

	float ex = Cglobal::instance().twist.linear.x - mmPs2twistx * Cglobal::instance().vx; //Fehler in x Richtung

	float Kpx = 1.2f;

	Cglobal::instance().twist.linear.x += Kpx * ex;

	float ey = Cglobal::instance().twist.linear.y - mmPs2twisty * Cglobal::instance().vy; //Fehler in y Richtung

	float Kpy = 1.0f;

	Cglobal::instance().twist.linear.y += Kpy * ey;

	ostr << "P: x:  " << Kpx * ex << endl;
	ostr << "P: y:  " << Kpy * ey << endl;

	float Ta = 0.05555555;

// D-Anteil:
	/*float Kdx = 0.05f; //Diese werte müssen noch angepasst werden und kommen dann noch in Cglobal als static Variable rein
	float Kdy = 0.05f;
	Cglobal::instance().twist.linear.x += Kdx * (ex - Cglobal::instance().exold) / Ta;
	Cglobal::instance().twist.linear.y += Kdy * (ey - Cglobal::instance().eyold) / Ta;

	ostr << "D: x:  " << Kdx * (ex - Cglobal::instance().exold) / Ta << endl;
	ostr << "D: y:  " << Kdy * (ey - Cglobal::instance().eyold) / Ta << endl;

	Cglobal::instance().exold = ex;
	Cglobal::instance().eyold = ey;*/

	Cglobal::instance().pub.publish(Cglobal::instance().twist);

	ostr << "linear.x: " << Cglobal::instance().twist.linear.x << endl;
	ostr << "linear.y: " << Cglobal::instance().twist.linear.y << endl << endl;
	ostr << "linear.z " << Cglobal::instance().twist.linear.z << endl;
	ostr << "angular.z " << Cglobal::instance().twist.angular.z << endl;
	ostr << "altd: " << Cglobal::instance().altd << endl;

	gettimeofday(&ac_time, NULL);
	ostr << "Zeit: " << ac_time.tv_sec - start.tv_sec << endl;
	ostr << endl << "-----------------------------------------------------------------" << endl;

	ROS_INFO(ostr.str().c_str());
	Cglobal::instance().of << ostr.str();


}

int main(int argc, char** argv) {
	ros::init(argc, argv, "follow_line2");

	ros::NodeHandle node_handle;
	Cglobal::instance().pub = node_handle.advertise < geometry_msgs::Twist
			> ("cmd_vel", 1000);
	ros::Subscriber sub = node_handle.subscribe("LinePos", 1000, handleLine);

	ros::Subscriber navdata = node_handle.subscribe("/ardrone/navdata", 1000,
			Math::navdataUpdate);

	gettimeofday(&start, NULL);

	while (!Cglobal::instance().end && ros::ok()) {
		ros::spinOnce();
	}

	Cglobal::destroy();
}
