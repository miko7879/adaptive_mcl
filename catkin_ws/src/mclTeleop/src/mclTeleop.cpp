#include "mclTeleop.h"

static struct termios oldt, newt;

/**
 * This function modifies the console so that it doesn't require enter 
 * key to be pressed on user input, and so that there is no console echo
 */
void swapConsole(){
	//Save the current terminal info
	tcgetattr(STDIN_FILENO, &oldt);
	//Make the new terminal identical to the old
	newt = oldt;
	//Disable input buffering and console echo
	newt.c_lflag &= ~(ICANON|ECHO);
	//Set the unbuffered input, no echo console as the current one
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

/**
 * This function restores the original console
 */
void restoreConsole(){
	//Restore proper console
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
