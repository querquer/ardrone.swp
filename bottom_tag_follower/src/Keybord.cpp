#include "Keybord.h"
#include "Global.h"

#include <termios.h>
#include <unistd.h>

/* @brief zum Steuern der Drone mit der Tastatur
 *
 * muss periodisch aufgerufen werden
 *
 * w,s,a,d : vorne, hinten, links, rechts
 * q: nach links drehen
 * e: nach rechts drehen
 * o: nach oben fliegen
 * l: nach unten fliegen
 * u: um 10% schneller werden
 * j: um 10% langsamer werden
 *
 */
void read()
{
	if(kbhit())
	{
	  char c = getch(); // Muss auf keine Eingabe warten, Taste ist bereits gedrückt
	  switch(c)
	  {
	  case 'w':
		  Cglobal::instance().twist.linear.x = 1 * Cglobal::instance().ges;
		  break;
	  case 's':
		  Cglobal::instance().twist.linear.x = -1 * Cglobal::instance().ges;
		  break;
	  case 'a':
		  Cglobal::instance().twist.linear.y = 1 * Cglobal::instance().ges;
		  break;
	  case 'd':
		  Cglobal::instance().twist.linear.y = -1 * Cglobal::instance().ges;
		  break;
	  case 'q':
		  Cglobal::instance().twist.angular.z = -1 * Cglobal::instance().ges;
		  break;
	  case 'e':
		  Cglobal::instance().twist.angular.z = 1 * Cglobal::instance().ges;
		  break;
	  case 'o':
		  Cglobal::instance().twist.linear.z = 1 * Cglobal::instance().ges;
		  break;
	  case 'l':
		  Cglobal::instance().twist.linear.z = -1 * Cglobal::instance().ges;
		  break;
	  case 'u':
		  Cglobal::instance().ges += 0.1*Cglobal::instance().ges;
		  break;
	  case 'j':
		  Cglobal::instance().ges += -0.1*Cglobal::instance().ges;
		  break;
	  case 3:
		  Cglobal::instance().end = true;
		  break;
	  }
	}
}


int kbhit(void) {
   struct termios term, oterm;
   int fd = 0;
   int c = 0;
   tcgetattr(fd, &oterm);
   memcpy(&term, &oterm, sizeof(term));
   term.c_lflag = term.c_lflag & (!ICANON);
   term.c_cc[VMIN] = 0;
   term.c_cc[VTIME] = 1;
   tcsetattr(fd, TCSANOW, &term);
   c = getchar();
   tcsetattr(fd, TCSANOW, &oterm);
   if (c != -1)
   ungetc(c, stdin);
   return ((c != -1) ? 1 : 0);
}

int getch()
{
   static int ch = -1, fd = 0;
   struct termios neu, alt;
   fd = fileno(stdin);
   tcgetattr(fd, &alt);
   neu = alt;
   neu.c_lflag &= ~(ICANON|ECHO);
   tcsetattr(fd, TCSANOW, &neu);
   ch = getchar();
   tcsetattr(fd, TCSANOW, &alt);
   return ch;
}
