#include "Keyboard.h"
#include "Global.h"

#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>

#include <sstream>

using namespace std;

namespace Keyboard
{

struct termios orig_termios;

/** @brief zum Steuern der Drone mit der Tastatur
 *
 * muss periodisch aufgerufen werden
 *
 *
 *
 * w,s,a,d : vorne, hinten, links, rechts
 *
 * q: nach links drehen
 *
 * e: nach rechts drehen
 *
 * o: nach oben fliegen
 *
 * l: nach unten fliegen
 *
 * u: um 10% schneller werden
 *
 * j: um 10% langsamer werden
 *
 */
void control()
{
	set_conio_terminal_mode();
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
	reset_terminal_mode();
}


void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

/** @brief gibt zurück, ob eine Taste gedrückt wurde
 */
int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}
/** @brief gibt zurück, welche Taste gedrückt wurde
 */
int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}


} //namespace Keyboard
