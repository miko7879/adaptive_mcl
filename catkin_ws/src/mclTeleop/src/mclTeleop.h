#include "stdio.h"
#include "stdlib.h"
#include <termios.h>          
#include <unistd.h> 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#define LIN_VEL 0.5
#define ANG_VEL 0.43633166666

void swapConsole(void);

void restoreConsole(void);
