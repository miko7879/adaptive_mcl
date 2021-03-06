/*
 * Localization.h
 *
 * This module contains all of the information necessary for localization
 *
 *  Created on: Feb 4, 2017
 *      Author: Michael Kogan and Garrett McDonald
 */

#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <time.h>
#include <stdlib.h>
#include <thread>
#include <mutex>

#define PI 3.14159265
#define ONE_OVER_SQRT_2PI 0.3989422804
#define MIN_RANGE 0.45
#define MAX_RANGE 4.0
#define MAX_RETRIES 5
#define ANG_DT 0.4
#define LIN_DT 0.5
#define X_CONVERGE 0.35
#define Y_CONVERGE 0.35
#define THETA_CONVERGE 10
#define MIN_PARTICLES 500
#define PARTICLE_LIMIT 1000
#define Z_099 0.83891
#define EPSILON 0.05
#define BIN_LENGTH 0.25
#define BIN_ANGLE 10
#define NUM_THREADS 4

using namespace std;

/**
 * Structure used to keep track of the position:
 * 	x -> The x coordinate
 * 	y -> The y coordinate
 * 	theta -> The angle which the position is facing
 */
struct pose{
	float x;
	float y;
	float theta;
};

/**
 * Structure used to keep track of the map:
 * 	map -> The occupancy grid
 * 	width -> The width
 * 	height -> The height
 *      resolution -> The resolution
 */
struct mapStruct{
	signed char **map;
	int width;
	int height;
	float resolution; 
};

/**
 * Structure used to represent a particle:
 * 	position -> The location of the particle
 * 	weight -> The weight associated with the particle
 */
struct particle{
	pose position;
	double weight;
};

/**
 * This type will allow us to point different functions to a generic localization function
 */
typedef vector<particle> (*RESAMPLING_FUNCTION) (vector<particle>, vector<double>);

/**
 * This function normalizes the given angle
 *
 * @param
 * 	angle -> The angle that we are normalizing
 * @return
 * 	float -> The normalized angle
 */
float normalizeAngle(float angle);

/**
 * This function calculates the distance given a dx, dy, and a theta
 *
 * @param
 * 	dx -> The x displacement
 * 	dy -> The y displacement
 * 	theta -> The angle at which we are looking
 * @return
 * 	float -> The distance
 */
float calculateDistance(int dx, int dy, float theta);

/**
 * This function projects the values of a range scan performed by the robot
 *
 * @param
 * 	p -> The position of the robot on the map
 * 	resolution -> The resolution of the map in cm
 * 	angleCount -> The amount of angles that we wish to generate (suggested is 61, to scan +- 30 degrees from direction in pose
 * 	minRange -> The minimum range of the sensor
 * 	maxRange -> The maximum range of the sensor
 * @return
 * 	vector<float> -> A vector the size of angleCount containing the projected distance values
 */
vector<float> generateView(pose p, float resolution, int angleCount, float startAngle, float angleResolution);
/**
 * This function generates a map given a set of points, a height, and a width, setting it to the global variable
 *
 * @param
 * 	points -> The set of points in row-ordinal form
 * 	h -> The height of the map
 * 	w -> The width of the map
 *      r -> The resolution of the map
 */
void generateMap(signed char points[], int h, int w, float r);

/**
 * This function gives the normal probability of result actual given a mean and a standard deviation
 *
 * @param
 * 	actual -> The result
 * 	mean -> The mean of the distribution
 * 	std_dev -> The standard deviation of the distribution
 * @return
 * 	float -> The probability of result actual in the distribution
 */
double normalPDF(float actual, float mean, float std_dev);

/**
 * This function determines the weight of an estimate using the projected view and the actual view
 *
 * @param
 * 	scan -> The reading that we measured from the environment
 * 	loc -> The location at which we believe we are at
 * @return
 * 	float -> The relative weight based on how likely our measurement is given our position
 */
double measurementModel(sensor_msgs::LaserScan scan, pose loc);

/**
 * This function applies the movement as defined by the linear and angular velocity, adding some variation to simulate the noise of the system
 *
 * @param
 * 	prevLoc -> The starting location
 * 	v -> The linear velocity
 * 	w -> The angular velocity
 * 	dt -> The amount of time for which we are applying the angular/linear velocities
 * @return
 * 	pose -> The new location after applying the movement
 */
pose movementModel(pose prevLoc, float v, float w);

/**
 * This function draws a sample from a normal distribution given variance b
 *
 * @param
 *  b -> The variance of the normal distribution
 * @return
 *  float -> The sample
 */
double sample(float b);

/**
 * This function takes the array from a /scan message and converts it to a 61-sized array (ANGLE_COUNT)
 *
 * @param
 * 	scan -> The raw data from the /scan message
 * @return
 * 	vector<float> -> The processed data, a 61-sized array of measurements
 */
vector<float> convertScan(vector<float> scan, int size);

/**
 * This function returns the map as a mapStruct
 *
 * @return
 * 	mapStruct -> The map and pertinent metadata
 */
mapStruct getMap(void);

/**
 * This function creates and distributes the number of particles specified by the NUM_PARTICLES constant
 *
 * @return
 * 	vector<particle> -> The list of particles
 */
vector<particle> distributeParticles(void);

/**
 * This function generates a random float specified by the boundaries
 *
 * @param
 * 	min -> The min value
 *	max -> The max value
 * @return
 * 	float -> The random value
 */
float generateRandomFloatBetween(float min, float max);

/**
 * This function generates a random double specified by the boundaries
 *
 * @param
 * 	min -> The min value
 *	max -> The max value
 * @return
 * 	float -> The random value
 */
double generateRandomDoubleBetween(double min, double max);

/**
 * This function will display the particles and the position of the robot
 *
 * @param
 *	particles -> The particles with which we are working
 *	ros::Publisher -> The publisher object for the particles
 *	ros::Publisher -> The publisher object for the position
 */
void displayResults(vector<particle> particles, ros::Publisher par, ros::Publisher pose);

/**
 * This function will generate a random position on the map
 *
 * @return
 *	pose -> The random position on the map
 */
pose generateRandomPositionOnMap();

/**
 * This function performs a basic uniform resampling of the particles
 *
 * @param
 *	particles -> The particles with which we are working
 *	cumsum -> The cumulative sum
 * @return
 *	vector<particles> -> The resamples particles
 */
vector<particle> uniformResample(vector<particle> particles, vector<double> cumsum);

/**
 * This function performs KLD resampling of the particles
 *
 * @param
 *	particles -> The particles with which we are working
 *	cumsum -> The cumulative sum
 * @return
 *	vector<particles> -> The resamples particles
 */
vector<particle> kldResample(vector<particle> particles, vector<double> cumsum);

/**
 * This function will return the number of particles
 *
 * @return
 *	int -> The total number of particles
 */
int getNumParticles(void);

/**
 * This function will initialize the bins to false
 */
void initBins(void);

/**
 * This function will reset the bins to false
 */
void resetBins(void);

/**
 * This function will run as a thread and perform part of the measurement update
 *
 * @param
 *	scan -> The scan that we are seeing
 *	particles -> The particles with which we will be working
 *	normFac -> The normalization factor
 *	threadNO -> The number of the thread
 */
void worker(sensor_msgs::LaserScan scan, vector<particle> & particles, double *normFac, int threadNO);

#endif /* LOCALIZATION_H_ */
