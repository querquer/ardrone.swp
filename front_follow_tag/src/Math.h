#ifndef MATH_H
#define MATH_H

#include "ar_recog/Tag.h"

/* @brief Klasse, die statische Funktionen enthält für z.B. berechen wo sich ein Tag bezüglich des Bildes befindet
 *
 */
class Cmath
{

  public:
	static void pixelDiff(float& x, float& y);
	static void center(const ar_recog::Tag& tag, float& cx, float& cy);
};

#endif //MATH_H

