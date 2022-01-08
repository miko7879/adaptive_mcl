/*
 * Localization.cpp
 *
 * This module contains the implementations of the functions in Localization.h
 *
 *  Created on: Feb 4, 2017
 *      Author: Michael Kogan and Garrett McDonald
 */

#include "Localization.h"

signed char **myMap;
bool ***bins;
int height, width, seqID = -1, noParticles = 1000, noBins;
float resolution;
mutex normFacMutex;

float normalizeAngle(float angle){
	float rtn = angle;
	while(rtn < 0)
		rtn += 2*PI;
	while(rtn > 2*PI)
		rtn -= 2*PI;
	return rtn;
}

float calculateDistance(int dx, int dy, float theta){
	if ((theta > PI / 4 && theta < 3 * PI / 4)
				|| (theta > 5 * PI / 4 && theta < 7 * PI / 4))
		return abs(dx/sin(theta))*resolution;
	return abs(dy/cos(theta))*resolution;
}

vector<float> generateView(pose p, float resolution, int angleCount, float startAngle, float angleResolution) {
	vector<float> scan(angleCount);
	float x = p.x, y = p.y, theta = 2*PI - p.theta;
	int startX = y / resolution, startY = x / resolution;
	for (int i = 0; i < angleCount; i++) {
		float offset, currentAngle = normalizeAngle(theta - i * angleResolution + startAngle),
				runningOffset = 0, directionX = sin(currentAngle), directionY =
						cos(currentAngle);
		int dx = 0, dy = 0;
		if ((currentAngle > PI / 4 && currentAngle < 3 * PI / 4)
				|| (currentAngle > 5 * PI / 4 && currentAngle < 7 * PI / 4)) {
			offset = fabs(1 / tan(currentAngle));
			while (myMap[startX + dx][startY + dy] == 0
					&& calculateDistance(dx, dy, currentAngle) < MAX_RANGE) {
				if (directionX > 0)
					dx--;
				else
					dx++;
				if (directionY > 0)
					runningOffset += offset;
				else
					runningOffset -= offset;
				dy = round(runningOffset);
			}
		} else {
			offset = fabs(tan(currentAngle));
			while (myMap[startX + dx][startY + dy] == 0
					&& calculateDistance(dx, dy, currentAngle) < MAX_RANGE) {
				if (directionX > 0)
					runningOffset -= offset;
				else
					runningOffset += offset;
				if (directionY > 0)
					dy++;
				else
					dy--;
				dx = round(runningOffset);
			}
		}
		float range = calculateDistance(dx, dy, currentAngle);
		if(range < MIN_RANGE || range > MAX_RANGE)
			range = MAX_RANGE;
		scan[i] = range;
	}
	return scan;
}

void generateMap(signed char points[], int h, int w, float r){
	myMap = 0, height = h, width = w, resolution = r;
	myMap = new signed char*[height];
	for(int i = 0; i < height; i++){
		myMap[i] = new signed char[width];
		for(int j = 0; j < width; j++){
			myMap[i][j] = points[i*width+j];
		}
	}
}

double normalPDF(float actual, float mean, float std_dev){
	double pow = (actual-mean)/std_dev;
	return ONE_OVER_SQRT_2PI/std_dev*exp(-0.5*pow*pow);
}

double measurementModel(sensor_msgs::LaserScan scan, pose loc){
	int angleCount = (scan.angle_max - scan.angle_min)/scan.angle_increment;
	vector<float> projected = generateView(loc, resolution, angleCount, scan.angle_max, scan.angle_increment), measured = convertScan(scan.ranges, angleCount);
	float w = 1, p;	
	for(int i = 0; i < angleCount; i++){
		p = 0;
		if(measured[i] == MAX_RANGE)
			p += 0.05;
		p += (normalPDF(measured[i], projected[i], 3*resolution)/normalPDF(projected[i], projected[i], 3*resolution))*0.95;
		w+=p;
	}
	return w;
}

pose movementModel(pose prevLoc, float v, float w){
	float v_prime = v + sample(0.3*v + 0.3*w), w_prime = w + sample(0.3*v+0.3*w), gamma = sample(0.05*v+0.05*w), divisor = v_prime/w_prime;
	float x = prevLoc.x - divisor*sin(prevLoc.theta) + divisor*sin(prevLoc.theta + w_prime*LIN_DT);
	float y = prevLoc.y + divisor*cos(prevLoc.theta) - divisor*cos(prevLoc.theta + w_prime*LIN_DT);
	float theta = normalizeAngle(prevLoc.theta + w_prime*ANG_DT + gamma*ANG_DT);
	int dx = round(x/resolution);
	int dy = round(y/resolution);
	if(dx >= width || dy >= height || dx < 0 || dy < 0){
		return generateRandomPositionOnMap();
	}	
	int noRetries = 0;
	while(myMap[dy][dx] != 0){
		if(noRetries == MAX_RETRIES){
			return generateRandomPositionOnMap();
		} else {
			x = prevLoc.x - divisor*sin(prevLoc.theta) + divisor*sin(prevLoc.theta + w_prime*LIN_DT);
			y = prevLoc.y + divisor*cos(prevLoc.theta) - divisor*cos(prevLoc.theta + w_prime*LIN_DT);
			dx = round(x/resolution);
			dy = round(y/resolution);
			noRetries++;
		}
	}
	pose nPose;
	nPose.x = x;
	nPose.y = y;
	nPose.theta = theta;
	return nPose;
}

double sample(float b){
	double s = 0;
	for(int i = 0; i < 12; i++){
		s += generateRandomDoubleBetween(-1,1);
	}
	return b/6*s;
}

vector<float> convertScan(vector<float> scan, int size){
	vector<float> parsed(size);
	for(int i = 0; i < size; i++){
		if(isnan(scan[i]) || scan[i] > MAX_RANGE)
			parsed[i] = MAX_RANGE;
		else
			parsed[i] = scan[i];
	}
	return parsed;
}

mapStruct getMap(){
  mapStruct m;
  m.map = myMap;
  m.height = height;
  m.width = width;
  m.resolution = resolution;
  return m;
}

vector<particle> distributeParticles(){
	vector<particle> particles = vector<particle>(noParticles);
	for(int i = 0; i < noParticles; i++){
		particle par;
		par.position = generateRandomPositionOnMap();
		par.weight = 1/noParticles;
		particles[i] = par;
	}
	return particles;
}

float generateRandomFloatBetween(float min, float max){
	return min + static_cast <float> ((rand()))/(static_cast <float> (RAND_MAX/(max-min)));
}

double generateRandomDoubleBetween(double min, double max){
	return min + static_cast <double> ((rand()))/(static_cast <double> (RAND_MAX/(max-min)));
}

void displayResults(vector<particle> particles, ros::Publisher par, ros::Publisher pose){
	float x = 0, y = 0, theta = 0;	
	seqID++;
	geometry_msgs::PoseArray parts;
	geometry_msgs::PoseStamped position;
	parts.header.seq = (unsigned int)seqID;
	parts.header.stamp = ros::Time::now();
	parts.header.frame_id = "/map";
	position.header.seq = (unsigned int) seqID;
	position.header.stamp = ros::Time::now();
	position.header.frame_id = "/map";
	for(int i = 0; i < noParticles; i++){
		geometry_msgs::Pose p;
		geometry_msgs::Quaternion quat;
		p.position.x = particles[i].position.x;
		p.position.y = particles[i].position.y;
		p.position.z = 0;
		quat.w = std::cos(particles[i].position.theta*0.5);
		quat.x = 0;
		quat.y = 0;
		quat.z = std::sin(particles[i].position.theta*0.5);
		p.orientation = quat;
		parts.poses.push_back(p);	
		x += particles[i].position.x;
		y += particles[i].position.y;
		theta += particles[i].position.theta;	
	}
	position.pose.position.x = x/noParticles;
	position.pose.position.y = y/noParticles;
	position.pose.position.z = 0;
	theta /= noParticles;
	position.pose.orientation.w = std::cos(theta*0.5);
	position.pose.orientation.x = 0;
	position.pose.orientation.y = 0;
	position.pose.orientation.z = std::sin(theta*0.5);
	par.publish(parts);
	if(noBins < 15)
		pose.publish(position);
}

pose generateRandomPositionOnMap(){
	pose pos;
	pos.x = generateRandomFloatBetween(0.0, (width-1)*resolution);
	pos.y = generateRandomFloatBetween(0.0, (height-1)*resolution);
	pos.theta = generateRandomFloatBetween(0.0, 2*PI);
	int dx = round(pos.x/resolution);
	int dy = round(pos.y/resolution);
	while(myMap[dy][dx] != 0){
		pos.x = generateRandomFloatBetween(0.0, (width-1)*resolution);
		pos.y = generateRandomFloatBetween(0.0, (height-1)*resolution);	
		dx = round(pos.x/resolution);
		dy = round(pos.y/resolution);
	}
	return pos;	
}

vector<particle> uniformResample(vector<particle> particles, vector<double> cumsum){
	vector<particle> newParticles = vector<particle>(noParticles);
	for(int i = 0; i < noParticles; i++){
		double guess = generateRandomDoubleBetween(0.0,1.0);
		int j = 0;
		while(cumsum[j] < guess){
			j++;			
		}
		particle p;
		p.position = particles[j].position;
		p.weight = 0;
		newParticles[i] = p;
	}
	return newParticles;
}

vector<particle> kldResample(vector<particle> particles, vector<double> cumsum){
	vector<particle> newParticles = vector<particle>();
	int newNoParticles = 0, kldBound = MIN_PARTICLES;
	noBins = 0;
	resetBins();
	do{
		double guess = generateRandomDoubleBetween(0.0,1.0);
		int j = 0;
		while(cumsum[j] < guess){
			j++;			
		}
		particle p;
		p.position = particles[j].position;
		p.weight = 0;
		newParticles.push_back(p);	
		int x = p.position.y/BIN_LENGTH, y = p.position.x/BIN_LENGTH, theta = ((2.0*PI - normalizeAngle(p.position.theta))*180.0/PI)/BIN_ANGLE; 
		if(bins[x][y][theta] == false){
			noBins++;
			bins[x][y][theta] = true;	
		}
		if(newNoParticles >= MIN_PARTICLES - 2 && noBins > 1){
				kldBound = 10.303*noBins + 85.245;
		}	
		newNoParticles++;
	}while((newNoParticles < MIN_PARTICLES || newNoParticles < kldBound) && newNoParticles < PARTICLE_LIMIT);
	noParticles = newNoParticles;
	cout<<"KLD: " << kldBound << "\n";
	cout<<"Bins: " << noBins << "\n";
	cout<<"New num particles: " << noParticles << "\n";
	return newParticles;
}

int getNumParticles(){
	return noParticles;
}

void initBins(){
	bins = new bool**[(int)(height*resolution/BIN_LENGTH + 1)];
	for(int i = 0; i < (int)(height*resolution/BIN_LENGTH + 1); i++){
		bins[i] = new bool*[(int)(width*resolution/BIN_LENGTH + 1)];
		for(int j = 0; j < (int)(width*resolution/BIN_LENGTH + 1); j++){
			bins[i][j] = new bool[(int)(360/BIN_ANGLE)];		
			for(int k = 0; k < (int)(360/BIN_ANGLE); k++){
				bins[i][j][k] = false;		
			}
		}
	}	
}

void resetBins(){
	for(int i = 0; i < (int)(height*resolution/BIN_LENGTH + 1); i++){
		for(int j = 0; j < (int)(width*resolution/BIN_LENGTH + 1); j++){
			for(int k = 0; k < (int)(360/BIN_ANGLE); k++){
				bins[i][j][k] = false;						
			}
		}
	}	
}

void worker(sensor_msgs::LaserScan scan, vector<particle> & particles, double *normFac, int threadNO){
	int lowerLimit, upperLimit;
	switch(threadNO){
		case 0:
			lowerLimit = 0;
			upperLimit = getNumParticles()/4;
			break;
		case 1:
			lowerLimit = getNumParticles()/4;
			upperLimit = getNumParticles()/2;
			break;
		case 2:
			lowerLimit = getNumParticles()/2;
			upperLimit = getNumParticles()*3/4;
			break;
		case 3:
			lowerLimit = getNumParticles()*3/4;
			upperLimit = getNumParticles();
			break;
	}
	for(int i = lowerLimit; i < upperLimit; i++){
		particles[i].weight = measurementModel(scan, particles[i].position); 
		normFacMutex.lock();
		*normFac+=particles[i].weight;
		normFacMutex.unlock();
	}
}

