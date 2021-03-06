/*
 * main.cpp
 *
 * This module contains the main function running the MCL algorithm
 *
 *  Created on: Feb 4, 2017
 *      Author: Michael Kogan and Garrett McDonald
 */

#include "Localization.h"

/**
 * This is the main function
 *
 * @param 
 * 	argc -> The number of arguments
 *	argv -> The arguments
 *
 * @return
 * 	0 -> Success
 * 	Others -> Error codes
 */
int main(int argc, char **argv)
{
	RESAMPLING_FUNCTION resample;
	//Get argument
	if(argc != 2){
		cout << "Usage: mcl <resampling algorithm number>\n";
		cout << "0: Normal resampling\n";
		cout << "1: KLD resampling\n";
		return 0;
	}
	char resamplingAlgoNumber = *(argv[1]) - 48;
	switch(resamplingAlgoNumber){
		case 0:
			cout<<"Normal Resampling\n";
			resample = &uniformResample;	
			break;
		case 1:
			cout <<"KLD Resampling\n";
			resample = &kldResample;
			break;
		default:
			cout << "Usage: mcl <resampling algorithm number>\n";
			cout << "0: Normal resampling\n";
			cout << "1: KLD resampling\n";
			return 0;
	}
	//Random seed
	srand((unsigned int)time(NULL));

	//Initialize the node, declare a node handle
	ROS_INFO("Initializing node \"mcl\"");	
  	ros::init(argc, argv, "mcl");
  	ros::NodeHandle n;

	//We will be publishing the particles as a pose array
	ros::Publisher particlePub = n.advertise<geometry_msgs::PoseArray>("/particles", 100);

	//We will be publishing the location as a pose
	ros::Publisher posePub = n.advertise<geometry_msgs::PoseStamped>("/position", 100);

	//Used to synchronize with teleop
	std_msgs::String s;
	ros::Publisher synch = n.advertise<std_msgs::String>("/Synch", 100);
  
 	//Read in map
	ROS_INFO("Getting map from topic \"/map\"");
 	nav_msgs::OccupancyGrid map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", n));
 	generateMap(&map.data[0], map.info.height, map.info.width, map.info.resolution);
	if(resamplingAlgoNumber == 1)
		initBins();

	//Distribute Particles
	ROS_INFO("Distributing particles");
	vector<particle> particles = distributeParticles();

	//Keep running node while ROS is good to go
 	while (ros::ok())
 	{	
		//Get next movement update
		geometry_msgs::Twist move = *(ros::topic::waitForMessage<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", n));

		//Apply movement update to aprticles while waiting for robot to finish moving
		for(int i = 0; i < getNumParticles(); i++){
			particles[i].position = movementModel(particles[i].position, move.linear.x, move.angular.z);
		}

 		//Get Sensor Update
		sensor_msgs::LaserScan scan = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n));
		
		//Normalization factor
		double normFac = 0;

		//Apply motion update and recalculate weights based on sensor update, and calculate the normalization factor
		thread measurementModelOne(worker, scan, ref(particles), &normFac, 0);
		thread measurementModelTwo(worker, scan, ref(particles), &normFac, 1);
		thread measurementModelThree(worker, scan, ref(particles), &normFac, 2);
		thread measurementModelFour(worker, scan, ref(particles), &normFac, 3);
		
		measurementModelOne.join();
		measurementModelTwo.join();
		measurementModelThree.join();
		measurementModelFour.join();
		
		//If nothing matches, redistribute
		if(normFac <= 50*getNumParticles()){
			particles = distributeParticles();
			synch.publish(s);
			continue;
		}

		ROS_INFO("NORMFAC: %f", normFac);

		//Create CDF
		vector<double> cumsum = vector<double>(getNumParticles());
		cumsum[0] = particles[0].weight;		
		for(int i = 1; i < getNumParticles(); i++){
			cumsum[i] = (cumsum[i-1] + particles[i].weight);	
		}
		
		for(int i = 0; i < getNumParticles(); i++){
			cumsum[i]/=normFac;
		}

		//Resample particles
		particles = resample(particles, cumsum);
		
		//Display Results in RViz
		displayResults(particles, particlePub, posePub);
		
		//Send synch message	
		synch.publish(s);
  	}
  	return 0;
}



