/**
 * @file Keyboard.h
 * @brief enthält Keyboard::control() zum steuern der Drone mit der Tastatur
 */
#ifndef KEYBOARD_H
#define KEYBOARD_H

/**
 * @namespace Keyboard
 * @brief enthält Keyboard::control() zum steuern der Drone mit der Tastatur
 */
namespace Keyboard
{
	void control();

	int kbhit();
	int getch();

	void set_conio_terminal_mode();
	void reset_terminal_mode();
}

#endif //KEYBOARD_H
