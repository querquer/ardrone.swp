#ifndef KEYBOARD_H
#define KEYBOARD_H

void read();

int kbhit();
int getch();

void set_conio_terminal_mode();
void reset_terminal_mode();

#endif //KEYBOARD_H
