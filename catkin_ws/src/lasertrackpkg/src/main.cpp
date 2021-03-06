/*
 * main.cpp
 *
 * This is the main module for mclTracker
 *
 * @param
 *	argc -> The number of arguments
 * 	argv -> The arguments
 *
 * @return
 *	0 -> Success
 *	Others -> Error codes
 *
 * 	Created on 12 March 2017
 *		Authors: Garrett McDonald and Michael Kogan
 */

#include "tracker.h"

int main(int argc, char **argv){
	
	//Initializes ROS node and declares velocity publisher
	ros::init(argc, argv, "mclTeleop");
  	ros::NodeHandle n;
	ros::Publisher velocity = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	//Declares a Twist message cmd and sets its permanent zeros
	geometry_msgs::Twist cmd;
	cmd.linear.y = 0;
	cmd.linear.z = 0;
	cmd.angular.x = 0;
	cmd.angular.y = 0;

	//Declares the variables to be used for determining movement based on laser location
	lasertrackpkg::laser_location_msg data;
	float xalign;
	float rotatevel;

	//Infinite loop reading laser location and driving motors towards it
	while(ros::ok){
		data = *(ros::topic::waitForMessage<lasertrackpkg::laser_location_msg>("/lasertrackpkg/laser_location", n));
		if(data.detected == 1){
			//Get Sensor Update
			sensor_msgs::LaserScan scan = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n));
			//Determine whether it is safe for the robot to drive forward
			bool drive = canDrive(scan.ranges);
			xalign = float(data.x)/float(data.xsize);
			rotatevel = 1-2*xalign;
			cmd.angular.z = 0;
			cmd.linear.x = 0;
			if (abs(rotatevel) < ANG_VEL){
				if(drive){
					cmd.linear.x = LIN_VEL;
				}
				else
					continue;
			}
			else if (rotatevel < 0)
				cmd.angular.z = - ANG_VEL;
			else
				cmd.angular.z = ANG_VEL; 
			velocity.publish(cmd);
			//ROS_INFO("Angular Vel: %f", cmd.angular.z);

			//Wait for a synch message from the localization algorithm declaring it is finished the movement update
			ros::topic::waitForMessage<std_msgs::String>("/Synch", n);
		}
	}
}
