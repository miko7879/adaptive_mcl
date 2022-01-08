/*
 * mclAutomove.h
 *
 * This module contains all of the information necessary for the mclAutomove ROS Package
 *
 * 	Created on 11 March 2017
 *		Authors: Garrett McDonald and Michael Kogan
 */
 
#include "stdio.h"
#include "stdlib.h"
#include <termios.h>          
#include <unistd.h> 
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <time.h>
#include <math.h>

#define LIN_VEL 0.5				//Robot Forward Velocity
#define ANG_VEL 0.43633166666	//Robot Turning Velocity
#define ANGLE_COUNT 639			//Size of the robot's returned scan array
#define MIN_RANGE 0.45			//Minimum scan range
#define MAX_RANGE 4.0			//Maximum scan range
#define ARC 0.1528				//Portion of angle count to be checked left and right for obstacles

using namespace std;

/*
 * This function determines if it is safe for the robot to drive forward
 *
 * @param
 *  raw -> the robots most recent laser scan return
 * @return
 *  bool -> returns true if the robot can advance forward, false otherwise
 */
bool canDrive(vector<float> raw);

/*
 * This function takes a laser scan array and converts it so that all irregular values are replaced by the max range
 *
 * @param
 *  raw -> a laser scan array
 * @return
 *  vector<float> -> a laser scan array with irregularities removed
 */
vector<float> convertScan(vector<float> scan);
