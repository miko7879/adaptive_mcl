/*
 * main.cpp
 *
 * This is the main module for the mclAutomove ROS package
 *
 * @param
 *	argc -> The number of arguments
 * 	argv -> The arguments
 *
 * @return
 *	0 -> Success
 *	Others -> Error codes
 *
 * 	Created on 11 March 2017
 *		Authors: Garrett McDonald and Michael Kogan
 */

#include "mclAutomove.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "mclTeleop");
  	ros::NodeHandle n;
	ros::Publisher velocity = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	
	//declare twist values, and set the permanent zero values
	geometry_msgs::Twist cmd;
	cmd.linear.y = 0;
	cmd.linear.z = 0;
	cmd.angular.x = 0;
	cmd.angular.y = 0;
	
	while(ros::ok){

		//Get Sensor Update
		sensor_msgs::LaserScan scan = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n));

		//Determine whether it is safe for the robot to drive forward
		bool drive = canDrive(scan.ranges);
		
		//Publish velocity to drive forward if possible, otherwise turn 
		if(drive){
			cmd.linear.x = LIN_VEL;
			cmd.angular.z = 0;
			velocity.publish(cmd);
		}else{
			cmd.linear.x = 0;
			cmd.angular.z = ANG_VEL;
			velocity.publish(cmd);
		}

		//Synch will be received when robot has finished current move
		ros::topic::waitForMessage<std_msgs::String>("/Synch", n);

	}	
	return 0;
}
