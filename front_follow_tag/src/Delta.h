#ifndef DELTA_H
#define DELTA_H

#include <time.h>
#include <stdio.h>

class Delta
{
public:
	Delta()
	{
		old_value = 0;
		old_time = 0;
		old_vel = 0;
	}
	float get_velocity(float new_value)
	{
		time_t new_time = time(NULL);
		//Geschwindigkeit = Weg/Zeit
		float vel = (new_value - old_value) / (new_time - old_time);

		//smooth Velocity
		old_vel = old_vel + (vel - old_vel) * 0.1;

		old_time = new_time;
		old_value = new_value;

		return old_value;
	}
private:

	float old_value;
	time_t old_time;
	float old_vel;
};

#endif //DELTA_H
