#ifndef DELTA_H
#define DELTA_H

#include <time.h>
#include <stdio.h>

class Delta
{
public:
	Delta();
	float get_velocity(float new_value);
private:

	float old_value;
	time_t old_time;
	float old_vel;
};

#endif //DELTA_H
