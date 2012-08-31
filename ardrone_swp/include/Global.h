/**
 * @file Global.h
 * @brief Singleton: enthält alle Globalen Variablen
 */
#ifndef GLOBAL_H
#define GLOBAL_H

#include "std_includes.h"


/** @class Cglobal
 * @brief Klasse, die in Form eines Singletons alle globalen Variablen enthält
 *
 */
class Cglobal
{
private:
      static Cglobal* m_instance;

      Cglobal();
      ~Cglobal() {}
      Cglobal(const Cglobal&) {}                            // nicht kopierbar
      Cglobal& operator=(const Cglobal&) { return *this; }  // nicht zuweisbar
   public:
      static Cglobal& instance();
      static void destroy();

   public:
      geometry_msgs::Twist twist; ///< Twist-Objekt, das die Bewegungsdaten zur Ansteuerung der Drone enthält

      geometry_msgs::Twist twist_old;  ///< enthält den Twist-Wert(ohne Regelung), der gesetzt wurde, als das Tag bzw. die Linie zuletzt gesehen wurde

      ros::Publisher pub;   ///< Publisher zum publishen des Twist Objekts

      int altd; ///< aktuelle Höhe

      time_t sinceNotSeen; ///< Zeit seit der das Tag das letzte mal gesehen wurde

      struct timeval sinceNoNavdataUpdate; ///< Zeit seit der das letzte Mal neue Navigationsdaten gesendet wurden

      bool seen; ///< enthält ob das Tag bzw. die Linie beim letzten Aufruf des Handlers gesehen wurde

      bool begin; ///< in follow_line wird, bevor der Linie gefolgt wird, nach oben geflogen, bis 0,9m erreicht wurden. begin gibt an ob die Drohne dabei noch am hochfliegen ist

      float vx; ///< aktuelle Geschwindigkeit in x-Richtung
      float vy; ///< aktuelle Geschwindigkeit in y-Richtung
      float vz; ///< aktuelle Geschwindigkeit in z-Richtung
      float roty; ///< aktuelle aktuelle Rotation um die y-Achse
      float rotx; ///< aktuelle aktuelle Rotation um die x-Achse

      float ges; ///< enthält wie stark auf Tastendrücke in Keyboard::control() eragiert werden soll

      bool end; ///< Gibt an ob in Keyboard::control() ctrl+c gedrückt wurde, falls true wird das Programm beendet

      //Bildgrößen
      static const int widthB = 160;  ///< Bildbreite der unteren Kamera
      static const int heightB = 120; ///< Bildhöhe der unteren Kamera
      static const int widthF = 320; ///< Bildbreite der Front-Kamera
      static const int heightF = 240; ///< Bildhöhe der Front-Kamera


      //Regelungsparameter
      static const float b_mmPs2twistx = 0.0002f; ///< Umrechnungsfaktor von mm/s in twist.linear.x (bei bottom_follow_tag): 0.0002, weil Drone in  x Richtung max 5m/s fliegt und der twist-Wert zwischen 0 und 1 liegt
  	  static const float b_mmPs2twisty = 0.0002f; ///< Umrechnungsfaktor von mm/s in twist.linear.y (bei bottom_follow_tag)

      static const float f_mmPs2twistx = 0.0002f; ///< Umrechnungsfaktor von mm/s in twist.linear.x (bei front_follow_tag)
  	  static const float f_mmPs2twisty = 0.0002f; ///< Umrechnungsfaktor von mm/s in twist.linear.y (bei front_follow_tag)

      static const float l_mmPs2twistx = 0.00015f; ///< Umrechnungsfaktor von mm/s in twist.linear.x (bei follow_line): geringer als bei den anderen beiden Applikationen, damit die Regelung nicht so empfindlich reagiert(bei einem höheren Wert ist die Drone beim Versuch still zu stehen teilweise nach hinten geflogen)
  	  static const float l_mmPs2twisty = 0.00025f; ///< Umrechnungsfaktor von mm/s in twist.linear.y (bei follow_line): höher als bei den anderen beiden Applikationen, damit die Regelung früher reagiert

  	  static const float b_Kpx = 2.5f; ///< Faktor für den Proportionalanteil in x-Richtung (bei bottom_follow_tag)
  	  static const float b_Kpy = 1.5f; ///< Faktor für den Proportionalanteil in y-Richtung (bei bottom_follow_tag)

  	  static const float f_Kpx = 1.5f; ///< Faktor für den Proportionalanteil in x-Richtung (bei front_follow_tag)
  	  static const float f_Kpy = 2.5f; ///< Faktor für den Proportionalanteil in y-Richtung (bei front_follow_tag)

  	  static const float l_Kpx = 1.2f; ///< Faktor für den Proportionalanteil in x-Richtung (bei follow_line)
  	  static const float l_Kpy = 1.0f; ///< Faktor für den Proportionalanteil in y-Richtung (bei follow_line)

  	  static const float Ta = 0.05555555f; ///< 1/18 = 0.0555... , wegen 18 Aufrufen pro Sekunde(ar_recog published 18 mal pro Sekunde)

  	  static const float b_Kdx = 0.05f; ///< Faktor für den Differentialanteil in x-Richtung (bei bottom_follow_tag)
  	  static const float b_Kdy = 0.05f; ///< Faktor für den Differentialanteil in y-Richtung (bei bottom_follow_tag)

  	  static const float f_Kdx = 0.05f; ///< Faktor für den Differentialanteil in x-Richtung (bei front_follow_tag)
  	  static const float f_Kdy = 0.05f; ///< Faktor für den Differentialanteil in y-Richtung (bei front_follow_tag)

      float exold; ///< Fehler beim letzten Aufruf in x-Richtung
      float eyold; ///< Fehler beim letzten Aufruf in y-Richtung
};

#endif //GLOBAL_H
