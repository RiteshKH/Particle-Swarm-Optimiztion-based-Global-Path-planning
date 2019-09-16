/*
 * Constants.h
 *
 * 
 * Author: Ritesh
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#endif /* CONSTANTS_H_ */

//PSO Core Parameters
const int SWARM_SIZE = 100; //Number of particles in Swarm
int NO_OF_ITERS = 500;      //Number of Iterations
double WMIN = 0.2;
double WMAX = 0.7;           //Inertial Weight
double C1 = 0.20, C2 = 0.60; //acceleration coefficients

const double START_X = 1000.0;        //Start X-Coordinate
const double START_Y = 1000.0;        //double Y-Coordinate
const double DESTINATION_X = -500.0;  //Destination X-Coordinate
const double DESTINATION_Y = -1000.0; //Destination Y-Coordinate

const double LOWER_BOUNDARY = -1500.0; //Search Space Lower bound
const double UPPER_BOUNDARY = 1500.0;  //Search Space Upper bound

const double V_MAX = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 10; //Max Particle Velocity
const double POS_MULTIPLE = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 2;
const double VEL_MULTIPLE = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 20;

const int NUM_OBSTACLE = 10;
const double R = 200.0;

const double TARGET_TOLERANCE = 150.0;                  //Tolerance for convergence
const double LOCAL_CONV_TOLERANCE = (NO_OF_ITERS / 20); //Tolerance for local minima convergence
