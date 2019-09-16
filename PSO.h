//============================================================================
// Name        : PSO.cpp(Ubuntu_18_Ritesh)
// Copyright   : Your copyright notice
// Description : Particle Swarm Implementation in C++, Ansi-style Header file
//============================================================================

#include <iostream>
#include <random>
#include <time.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <string>

#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include "Constants.h"

using namespace std;

// Structure to store positions and velocities
struct Coord
{
    double x;
    double y;
    double velx;
    double vely;
};

// Generate random numbers
double *generate_random(int swarm_size, double lower_bound, double upper_bound, int i)
{
    srand(time(NULL));
    // std::default_random_engine re;
    std::mt19937 re;
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count() + i++;
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    re.seed(ss);
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);

    double *a_random_array = new double[swarm_size];
    const int nSimulations = swarm_size;
    for (int j = nSimulations; j--;)
    {
        double a_random_double = unif(re);
        a_random_array[j] = a_random_double;
    }
    return a_random_array;
}

//============================================================================================================================//
// Implementing Fitness function update from "Multi-Objective PSO-based Algorithm for Robot Path Planning"
// Find shortest path
double shortfit(Coord input, Coord goal)
{
    double distance;
    distance = sqrt(pow((int)input.x - (int)goal.x, 2.0) + pow((int)input.y - (int)goal.y, 2.0));
    return distance;
}
// Find smoothest path
double smoothfit(Coord input, Coord goal, Coord gbest)
{
    double num = ((input.x - goal.x) * (gbest.x - goal.x)) + ((input.y - goal.y) * (gbest.y - goal.y));
    double den = sqrt(pow((input.x - goal.x), 2.0) + pow((input.y - goal.y), 2.0)) * sqrt(pow((gbest.x - goal.x), 2.0) + pow((gbest.y - goal.y), 2.0));
    double smooth = acos(num / den);
    return smooth;
}
// Overall fitness function
double fitness(Coord input, Coord gbest)
{
    Coord target; // Goal to reach
    target.x = DESTINATION_X;
    target.y = DESTINATION_Y;
    double distance = shortfit(input, target);
    double smooth = smoothfit(input, target, gbest);
    double alpha1 = 1, alpha2 = 1;
    double total_fitness = alpha1 * distance + alpha2 * smooth;
    return (1 / total_fitness);
}

// shortest distance from obstacle
double obsdistance(Coord input1, Coord input0, double center_x, double center_y, double radius)
{
    double slope = (input1.y - input0.y) / (input1.x - input0.x);
    double num = fabs((slope * center_x) - center_y + input0.y - (slope * input0.x));
    double den = sqrt(1 + pow(slope, 2.0));
    double obsDist = ((num / den) - radius);
    return obsDist;
}

Coord newparticle(Coord input1, Coord input0, double center_x, double center_y, double radius, int seed)
{
    double thresh = radius * 0.3;
    Coord newPart1, newPart2, newPartFinal;
    double slope = (input1.y - input0.y) / (input1.x - input0.x);
    double invslope = -1 * (1 / slope);
    double obs = obsdistance(input1, input0, center_x, center_y, radius);
    double rn = generate_random(1, 0, 1, seed)[0];
    double safedistance = radius + (rn * thresh);

    double newX1 = (safedistance / sqrt(1 + pow(invslope, 2.0))) + center_x;
    double newY1 = ((safedistance * invslope) / sqrt(1 + pow(invslope, 2.0))) + center_y;
    newPart1.x = newX1;
    newPart1.y = newY1;

    double newX2 = -1 * (safedistance / sqrt(1 + pow(invslope, 2.0))) + center_x;
    double newY2 = -1 * ((safedistance * invslope) / sqrt(1 + pow(invslope, 2.0))) + center_y;
    newPart2.x = newX2;
    newPart2.y = newY2;
    if (shortfit(input0, newPart1) <= shortfit(input0, newPart2))
    {
        newPartFinal = newPart1;
    }
    else
    {
        newPartFinal = newPart2;
    }
    return newPartFinal;
}
//===========================================================================================================================//

// Write to csv
void write_to_csv(vector<vector<double>> particle_pos_x, vector<vector<double>> particle_pos_y, vector<Coord> global_pos,
                  vector<vector<double>> local_pos_x, vector<vector<double>> local_pos_y, int swarm_size, int counter,
                  double center_x[], double center_y[], double r, int num_obstacles, vector<Coord> path_coord)
{
    // create an ofstream for the file output
    std::ofstream outputFile;
    ofstream fs;
    // create a name for the file output
    string filename = "graph_data.csv";
    // create and open the .csv file
    outputFile.open(filename.c_str());

    // write data to the file
    // Columns: particles
    // Rows: iterations
    for (int i = 0; i < swarm_size; i++)
    {
        outputFile << "Particle_x" << i + 1 << ","
                   << "Particle_y" << i + 1 << ",";
    }
    outputFile << "global_position_x"
               << ","
               << "global_position_y"
               << ",";
    outputFile << "path_x"
               << ","
               << "path_y"
               << ",";
    for (int i = 0; i < swarm_size; i++)
    {
        outputFile << "local_position_x" << i + 1 << ","
                   << "local_position_y" << i + 1 << ",";
    }
    outputFile << "num_obstacles"
               << ",";
    for (int i = 0; i < num_obstacles; i++)
    {
        outputFile << "center_x" << i + 1 << ","
                   << "center_y" << i + 1 << ","
                   << "r"
                   << ",";
    }

    outputFile << "\n";

    for (int iter = 0; iter < counter; iter++)
    {
        for (int i = 0; i < swarm_size; i++)
        {
            outputFile << particle_pos_x[iter][i] << "," << particle_pos_y[iter][i] << ",";
        }
        if (iter == counter - 1)
        {
            outputFile << global_pos[iter + 1].x << "," << global_pos[iter + 1].y << ",";
        }
        else
        {
            outputFile << global_pos[iter].x << "," << global_pos[iter].y << ",";
        }
        outputFile << path_coord[iter].x << "," << path_coord[iter].y << ",";

        for (int i = 0; i < swarm_size; i++)
        {
            outputFile << local_pos_x[iter][i] << "," << local_pos_y[iter][i] << ",";
        }
        outputFile << num_obstacles << ",";
        for (int i = 0; i < num_obstacles; i++)
        {
            outputFile << center_x[i]
                       << ","
                       << center_y[i]
                       << ","
                       << r
                       << ",";
        }

        outputFile << "\n";
    }
    // close the output file
    outputFile.close();
    cout << "Values written to csv file" << endl;
}

// Display at each iteration
void epochDisplay(int iter_ctr, double global_best_fit, Coord global_best_pos)
{
    cout << "Iteration :" << iter_ctr << ".........." << endl;
    cout << "New Global Best fitness: " << global_best_fit * pow(10, 3) << endl;
    cout << "New Global Best Position: (" << global_best_pos.x << "," << global_best_pos.y << ")" << endl;
}

// ========================================================================================================================================//
// Implementing obstacles

// Circular obstacle
bool inCircle(double x, double y, double center_x, double center_y, double r)
{
    double radius = sqrt(pow(x - center_x, 2.0) + pow(y - center_y, 2.0));
    if (radius <= 1.2 * r)
        return true;
    else
        return false;
}

// OBSTACLE AVOIDANCE MODULE
bool obstacle_avoidance(int num_obstacle, Coord &input1, Coord input0, double center_x[], double center_y[], double r,
                        double thresh, int seed, double lower_boundary, double upper_boundary)
{
    vector<bool> validinit(num_obstacle);
    vector<bool> obsdistarrinit(num_obstacle);
    bool incircle[num_obstacle];
    bool final_val_init = false;
    for (int k = num_obstacle; k--;)
    {
        double obsdist = obsdistance(input1, input0, center_x[k], center_y[k], r);
        if ((obsdist <= thresh))
        {
            // cout << "Start Path crossing obstacle, Infeasible.Create particle near boundary of obstacle" << endl;
            Coord newPart = newparticle(input1, input0, center_x[k], center_y[k], r, seed++);
            if (isnan(newPart.x) == 1 || isnan(newPart.y) == 1)
            {
                input1 = input0;
            }
            else
            {
                input1.x = newPart.x;
                input1.y = newPart.y;
            }
            // if (input1.x > upper_boundary)
            //     input1.x = upper_boundary;
            // if (input1.y > upper_boundary)
            //     input1.y = upper_boundary;
            // if (input1.x < lower_boundary)
            //     input1.x = lower_boundary;
            // if (input1.y < lower_boundary)
            //     input1.y = lower_boundary;
        }

        incircle[k] = inCircle(input1.x, input1.y, center_x[k], center_y[k], r);
        if (incircle[k] == true)
            validinit[k] = true;
        else
            validinit[k] = false;
    }
    // Check if all generated particles path are free of obstacles
    for (int k = num_obstacle; k--;)
    {
        double obsdist = obsdistance(input1, input0, center_x[k], center_y[k], r);
        if (obsdist <= thresh)
            obsdistarrinit[k] = true;
        else
            obsdistarrinit[k] = false;
    }
    // Check if all particles are free of obstacles
    if (all_of(validinit.begin(), validinit.end(), [](bool io) { return !io; }) && all_of(obsdistarrinit.begin(), obsdistarrinit.end(), [](bool io) { return !io; }))
    {
        final_val_init = false;
        // cout << "Particles free. \n ";
    }
    else
    {
        final_val_init = true;
        // cout << "Initial particles in obstacle, re-initializing \n ";
    }

    return final_val_init;
}
