/**
 * @file Delta.h
 * @brief Klasse, zur Berechnung der Geschwindigkeit (wird nicht mehr verwendet)
 */
#ifndef DELTA_H
#define DELTA_H

#include <time.h>
#include <stdio.h>

/**
 * @class Delta
 * @brief Klasse, zur Berechnung der Geschwindigkeit (wird nicht mehr verwendet)
 *
 * Übergeben wird der Weg(bzw. Distanzwert), Berechnung erfolgt nach Weg/Zeit
 */
class Delta
{
public:
	Delta();
	float get_velocity(float new_value);
private:

	float old_value; ///< enthält den letzen übergebenen Wert
	time_t old_time; ///< enthält die Zeit, seitdem letzten Aufruf von float Delta::get_velocity(float new_value)
	float old_vel; ///< enthält den letzten zurückgegebenen Wert
};

#endif //DELTA_H
