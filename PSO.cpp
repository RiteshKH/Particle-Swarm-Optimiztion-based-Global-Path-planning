//============================================================================
// Name        : PSO.cpp(Ubuntu_18_Ritesh)
// Author      : Ritesh
// Version     : v5
// Copyright   : Your copyright notice
// Description : Particle Swarm Implementation in C++, Ansi-style
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
#include <array>
#include "PSO.h"

using namespace std;

int main()
{
    //Initial and Control Parameters
    int swarm_size = 100;  //Swarm Size
    int no_of_iters = 500; //Number of Iterations
    double w;              //Inertial Weight
    double wmin = 0.2;
    double wmax = 0.7;
    double c1 = 2, c2 = 2; //acceleration coefficients
    Coord target;          // Goal to reach
    target.x = -500.0;
    target.y = -1000.0;
    Coord start;
    start.x = 1000.0;
    start.y = 1000.0;

    //Variables Initialization
    //Random Numbers used for updating position and velocity of particles
    // Positions // Velocities

    int seed = 1; // Variable to induce different timeseeds in each call for different random number generation
    double lower_bound = -1;
    double upper_bound = 1;

    double *position_x = generate_random(swarm_size, lower_bound, upper_bound, seed++);
    double *position_y = generate_random(swarm_size, lower_bound, upper_bound, seed++);
    double *velocity_x = generate_random(swarm_size, lower_bound, upper_bound, seed++);
    double *velocity_y = generate_random(swarm_size, lower_bound, upper_bound, seed++);

    // Structure to store random data for position and velocities
    Coord States[swarm_size];
    for (int i = swarm_size; i--;)
    {
        States[i].x = position_x[i];
        States[i].y = position_y[i];
        States[i].velx = velocity_x[i];
        States[i].vely = velocity_y[i];
    }

    // New Initializations
    Coord particle_states[swarm_size];   //position and velocities of particles
    double particle_fitness[swarm_size]; //fitness of particles
    Coord local_best_pos[swarm_size];    //local best position of each particle across iterations
    double local_best_fit[swarm_size];   //local best fitness of each particle across iterations
    Coord global_best_pos;               //global best position across all particles across iterations
    double global_best_fit;
    // Boundary conditons for particle initialisations
    double lower_boundary = -1500.0;
    double upper_boundary = 1500.0;

    //Initialize all arrays to 0
    global_best_pos.x = start.x;
    global_best_pos.y = start.y;

    int cols = swarm_size + 1;
    int rows = no_of_iters + 1;
    int initial_value = 0;

    int counter = 1;
    vector<Coord> global_pos(rows);
    global_pos[0].x = global_best_pos.x;
    global_pos[0].y = global_best_pos.y;

    vector<vector<double>> particle_pos_x;
    particle_pos_x.resize(rows, vector<double>(cols, initial_value));
    vector<vector<double>> particle_pos_y;
    particle_pos_y.resize(rows, vector<double>(cols, initial_value));

    vector<Coord> path_coord;

    vector<vector<double>> local_pos_x;
    local_pos_x.resize(rows, vector<double>(cols, initial_value));
    vector<vector<double>> local_pos_y;
    local_pos_y.resize(rows, vector<double>(cols, initial_value));

    const double V_max = (upper_boundary - lower_boundary) / 10;
    global_best_fit = fitness(global_best_pos, target, global_pos[0]);
    double pos_multiple = (upper_boundary - lower_boundary) / 2;
    double vel_multiple = (upper_boundary - lower_boundary) / 20;

    // ==================================================================================//
    // Initialize obstacle parameters (Obstacles at random positions)
    const int num_obstacle = 7;
    bool incircle[num_obstacle];
    double r = 200.0;
    // double center_x[num_obstacle] = {-600, -300, 550, 600, 250};
    // double center_y[num_obstacle] = {-100, 500, 500, -200, -1000};
    double center_x[num_obstacle];
    double center_y[num_obstacle];
    for (int i = num_obstacle; i--;)
    {
    init:
        int c_x = (pos_multiple - 200) * (generate_random(1, -1, 1, seed++)[0]);
        int c_y = (pos_multiple - 200) * (generate_random(1, -1, 1, seed++)[0]);
        bool stinobs = inCircle(start.x, start.y, c_x, c_y, r);
        bool tarinobs = inCircle(target.x, target.y, c_x, c_y, r);
        if (!stinobs && !tarinobs)
        {
            center_x[i] = c_x;
            center_y[i] = c_y;
        }
        else
        {
            cout << "start or target in obstacle....." << endl;
            goto init;
        }
    }
    // ================================================================================================================================================//

    /**
	 * First Iteration. Initialize swarm with Random Position and Velocity vectors for Each Particle.
	 */
    double thresh = (r * 0.1);
    for (int i = swarm_size; i--;)
    {
        bool final_val_init = false;
        int ct = 500;
        do
        {
            particle_states[i].x = pos_multiple * (States[i].x);
            particle_states[i].y = pos_multiple * (States[i].y);
            double new_x = generate_random(1, 0, 1, seed++)[0];
            States[i].x = new_x;
            double new_y = generate_random(1, 0, 1, seed++)[0];
            States[i].y = new_y;
            if (particle_states[i].x > upper_boundary)
                particle_states[i].x = upper_boundary;
            if (particle_states[i].y > upper_boundary)
                particle_states[i].y = upper_boundary;
            if (particle_states[i].x < lower_boundary)
                particle_states[i].x = lower_boundary;
            if (particle_states[i].y < lower_boundary)
                particle_states[i].y = lower_boundary;

            //===================================================================================================//
            // OBSTACLE AVOIDANCE MODULE
            final_val_init = obstacle_avoidance(num_obstacle, particle_states[i], start, center_x, center_y, r, thresh, seed, lower_boundary, upper_boundary);
            //===================================================================================================//
            ct--;

        } while ((final_val_init) && (ct != 0));

        particle_states[i].velx = vel_multiple * (States[i].x);
        particle_states[i].vely = vel_multiple * (States[i].y);

        // Representing fitness function as the distance between the particles and local best
        particle_fitness[i] = fitness(particle_states[i], target, global_pos[0]);
        local_best_pos[i].x = particle_states[i].x;
        local_best_pos[i].y = particle_states[i].y;
        local_best_fit[i] = particle_fitness[i];

        if (local_best_fit[i] < global_best_fit)
        {
            global_best_pos.x = particle_states[i].x;
            global_best_pos.y = particle_states[i].y;
            global_best_fit = local_best_fit[i];
        }

        // Write to csv data
        local_pos_x[1][i] = local_best_pos[i].x;
        local_pos_y[1][i] = local_best_pos[i].y;
        global_pos[1].x = global_best_pos.x;
        global_pos[1].y = global_best_pos.y;
        particle_pos_x[1][i] = particle_states[i].x;
        particle_pos_y[1][i] = particle_states[i].y;
    }

    // Initial Display
    epochDisplay(1, global_best_fit, global_best_pos);

    /**
	 * Calculations from Second iteration Onwards
	 */
    int flag = 1, cnt = 0;
    for (int iter_ctr = 2; iter_ctr <= no_of_iters; iter_ctr++)
    {

        //Updating Velocity - Applying Psycho-Social Criteria
        for (int i = swarm_size; i--;)
        {
            bool final_val = false;
            int ct = 500;
            do
            {
                double rp = generate_random(1, 0, 1, seed++)[0];
                double rg = generate_random(1, 0, 1, seed++)[0];
                // w = wmax - (((wmax - wmin) / (no_of_iters)) * iter_ctr);
                // following update from "Intelligent Vehicle Global Path Planning Based on Improved Particle Swarm Optimization"
                double c = 2.3;
                w = wmin + (wmax - wmin) * exp(-pow(((c * iter_ctr) / no_of_iters), 2.0));
                // cout << "w value : " << w << endl;
                particle_states[i].velx = (w * particle_states[i].velx + c1 * rp * (local_best_pos[i].x - particle_states[i].x)) + c2 * rg * (global_best_pos.x - particle_states[i].x);
                particle_states[i].vely = (w * particle_states[i].vely + c1 * rp * (local_best_pos[i].y - particle_states[i].y)) + c2 * rg * (global_best_pos.y - particle_states[i].y);
                if (particle_states[i].velx > V_max)
                    particle_states[i].velx = V_max;
                if (particle_states[i].vely > V_max)
                    particle_states[i].vely = V_max;
                if (particle_states[i].velx < -V_max)
                    particle_states[i].velx = -V_max;
                if (particle_states[i].vely < -V_max)
                    particle_states[i].vely = -V_max;

                particle_states[i].x = particle_states[i].x + particle_states[i].velx;
                particle_states[i].y = particle_states[i].y + particle_states[i].vely;
                // if (particle_states[i].x > upper_boundary)
                //     particle_states[i].x = upper_boundary;
                // if (particle_states[i].y > upper_boundary)
                //     particle_states[i].y = upper_boundary;
                // if (particle_states[i].x < lower_boundary)
                //     particle_states[i].x = lower_boundary;
                // if (particle_states[i].y < lower_boundary)
                //     particle_states[i].y = lower_boundary;

                // ==============================================================================================================================================================//

                // OBSTACLE AVOIDANCE MODULE
                final_val = obstacle_avoidance(num_obstacle, particle_states[i], global_best_pos, center_x, center_y, r, thresh, seed, lower_boundary, upper_boundary);
                //===================================================================================================//
                ct--;

            } while ((final_val) && (ct != 0));

            // Write to csv data
            particle_pos_x[iter_ctr][i] = particle_states[i].x;
            particle_pos_y[iter_ctr][i] = particle_states[i].y;
        }

        //Assign Local & Global Best position & Fitness
        //For each particle evaluate fitness
        //If fitness(xt) > fitness(gbest) then gbest=xt
        //If fitness(xt) > fitness(pbest) then pbest=xt
        for (int i = swarm_size; i--;)
        {
            //Set Particle Fitness
            particle_fitness[i] = fitness(particle_states[i], target, global_pos[iter_ctr - 1]);

            // Updating local/particle best position
            if (fitness(particle_states[i], target, global_pos[iter_ctr - 1]) < local_best_fit[i])
            {
                local_best_pos[i].x = particle_states[i].x;
                local_best_pos[i].y = particle_states[i].y;
                local_best_fit[i] = fitness(local_best_pos[i], target, global_pos[iter_ctr - 1]);
                // Updating global best position
                if (local_best_fit[i] < global_best_fit)
                {
                    global_best_pos.x = local_best_pos[i].x;
                    global_best_pos.y = local_best_pos[i].y;
                    global_best_fit = fitness(global_best_pos, target, global_pos[iter_ctr - 1]);

                    global_best_pos.velx = particle_states[i].velx;
                    global_best_pos.vely = particle_states[i].vely;
                }
            }
            // Write to csv data
            local_pos_x[iter_ctr][i] = local_best_pos[i].x;
            local_pos_y[iter_ctr][i] = local_best_pos[i].y;
            global_pos[iter_ctr].x = global_best_pos.x;
            global_pos[iter_ctr].y = global_best_pos.y;
        }

        if (iter_ctr > 3)
        {
            if (((global_pos[iter_ctr].x - global_pos[iter_ctr - 1].x) < 0.2) && ((global_pos[iter_ctr].x - global_pos[iter_ctr - 2].x) < 0.2) && ((global_pos[iter_ctr].x - global_pos[iter_ctr - 3].x) < 0.2) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 1].y) < 0.2) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 2].y) < 0.2) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 3].y) < 0.2))
            {
                cout << "Local Minima detected: " << endl;
                // Implementing Shortest Path Planning PSO
                // (Applies a slight perturbation to the Global best particles so that it doesn't get stuck in a local minima.
                //  The particles are updated by the given velocity formula)
                //Check if particles are within boundary
                bool final_val = false;
                vector<bool> valid;
                int cs = 500;
                do
                {
                    double alpha = 0.2, beta = 0.3; // Arbitrarily generated constants
                    double r3 = generate_random(1, 0, 1, seed++)[0];
                    global_best_pos.velx = (alpha * global_best_pos.velx) + (beta * r3);
                    global_best_pos.vely = (alpha * global_best_pos.vely) + (beta * r3);
                    Coord gbest;
                    gbest.x = global_best_pos.x + global_best_pos.velx;
                    gbest.y = global_best_pos.y + global_best_pos.vely;
                    double gfit = fitness(gbest, target, global_pos[iter_ctr - 1]);
                    global_best_pos = gbest;
                    global_best_fit = gfit;

                    //===================================================================================================//
                    // OBSTACLE AVOIDANCE MODULE::
                    final_val = obstacle_avoidance(num_obstacle, global_best_pos, global_best_pos, center_x, center_y, r, thresh, seed, lower_boundary, upper_boundary);
                    //===================================================================================================//

                    cs--;
                } while (final_val && cs != 0);
            }
        }

        // All Displays
        epochDisplay(iter_ctr, global_best_fit, global_best_pos);

        // Check if converged on goal
        counter++;
        if (shortfit(global_best_pos, target) <= 150.0)
        {
            cout << "\n\nReached near goal. SUCCESS " << endl;
            flag = 0;
            break;
        }
        else if (iter_ctr >= (no_of_iters / 20))
        {
            if (((global_pos[iter_ctr].x - global_pos[iter_ctr - 1].x) < 0.1) && ((global_pos[iter_ctr].x - global_pos[iter_ctr - 2].x) < 0.1) && ((global_pos[iter_ctr].x - global_pos[iter_ctr - 3].x) < 0.1) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 1].y) < 0.1) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 2].y) < 0.1) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 3].y) < 0.1))
            {
                cout << "\n\nCouldn't find a feasible path / Converged at a local minima. FAILED" << endl;
                flag = 1;
                break;
            }
        }
    }

    // Optimising Path generated (Optional)
    Coord path_cd;
    path_cd.x = start.x;
    path_cd.y = start.y;
    path_coord.insert(path_coord.begin() + path_coord.size(), path_cd);
    for (int iter_ctr = 0; iter_ctr < counter; iter_ctr++)
    {
        if (iter_ctr > 3)
        {
            if ((shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr - 1], target)) &&
                (shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr - 2], target)) &&
                (shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr - 3], target)))
            {
                path_coord.insert(path_coord.begin() + path_coord.size(), global_pos[iter_ctr]);
            }
            else
            {
                path_coord.insert(path_coord.begin() + path_coord.size(), global_pos[iter_ctr - 1]);
            }
        }
        else
        {
            path_coord.insert(path_coord.begin() + path_coord.size(), global_pos[0]);
        }
    }

    cout << endl
         << "Final Global Best Position: (" << global_best_pos.x << "," << global_best_pos.y << ")" << endl;

    // Writing to csv all the data for plotting
    write_to_csv(particle_pos_x, particle_pos_y, global_pos, local_pos_x, local_pos_x, swarm_size, counter, center_x, center_y, r, num_obstacle, path_coord);
    cout << "Run Successful" << endl;

    return 0;
}

// Pseudo code
/*
 Start
 Initialize swarm with Random Position (x0) and velocity vectors (v0)
 for Each Particle
 Evaluate Fitness
 If fitness(xt) > fitness(gbest)
 gbest=xt
 If fitness(xt) > fitness(pbest)
 pbest=xt
 Update Velocity
 v(t+1)=W*vt + c1*rand(0,1)*(pbest-xt)+c2*rand(0,1)*gbest-xt)
 Update Position
 X(t+1) = Xt+V(t+1)

 Go to next particle

 If Terminate
 gbest is the output
 Else goto for Each Particle
 */
