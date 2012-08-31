/** @file Math.h
 *  @brief enthält Funktionen für das Berechen wo sich ein Tag bezüglich des Bildes befindet und die Regelung
 */
#ifndef MATH_H
#define MATH_H

#include "ar_recog/Tag.h"
#include "ardrone_brown/Navdata.h"

/** @namespace Math
 *  @brief enthält Funktionen für das Berechen wo sich ein Tag bezüglich des Bildes befindet und die Regelung
 */
namespace Math
{
	void pixelDiffBottom(float& x, float& y);
	void pixelDiffFront(float& x, float& y);
	void centerBottom(const ar_recog::Tag& tag, float& cx, float& cy);
	void centerFront(const ar_recog::Tag& tag, float& cx, float& cy);
	void navdataUpdate(const ardrone_brown::Navdata::ConstPtr& navdata);
	void bottom_regulation();
	void front_regulation();
	void line_regulation();
};

#endif //MATH_H

