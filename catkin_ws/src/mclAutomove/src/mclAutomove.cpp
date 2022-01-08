/*
 * mclAutomove.cpp
 *
 * This module contains the implementations of the functions in mclAutomove.h
 *
 * 	Created on 11 March 2017
 *		Authors: Garrett McDonald and Michael Kogan
 */

#include "mclAutomove.h"

bool canDrive(vector<float> raw){
	float dist = 2*MIN_RANGE;
	vector<float> scan = convertScan(raw);
	float center = scan[(ANGLE_COUNT+1)/2];
	float left = scan[0];
	float right = scan[ANGLE_COUNT-1];
	bool drive = false;
	int i;
	ROS_INFO("Forward Scan: %f", center);
	if(left == MAX_RANGE){
		for(i = 0; i <= ARC*ANGLE_COUNT;i++){
			if(scan[i] > MIN_RANGE && scan[i] < MAX_RANGE){
				left = scan[i];
				break;			
			}
		}
	}
	ROS_INFO("Left Scan: %f", left);
	if(right == MAX_RANGE){
		for(i = ANGLE_COUNT-1; i >= (ANGLE_COUNT - (ARC*ANGLE_COUNT));i--){
			if(scan[i] > MIN_RANGE && scan[i] < MAX_RANGE){
				right = scan[i];
				break;			
			}
		}
	}
	ROS_INFO("Right Scan: %f", right);
	if(center > dist && left < MAX_RANGE && left > 1.5*MIN_RANGE && right < MAX_RANGE && right > 1.5*MIN_RANGE){
		drive = true;
	}
	return drive;
}

vector<float> convertScan(vector<float> scan){
	vector<float> parsed(ANGLE_COUNT);
	for(int i = 0; i < ANGLE_COUNT; i++){
		if(isnan(scan[i]) || scan[i] > MAX_RANGE)
			parsed[i] = MAX_RANGE;
		else
			parsed[i] = scan[i];
	}
	return parsed;
}
