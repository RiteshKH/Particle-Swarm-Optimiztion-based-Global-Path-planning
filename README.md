## [Particle-Swarm-Optimiztion-based-Global-Path-planning](https://github.com/RiteshKH/Particle-Swarm-Optimiztion-based-Global-Path-planning)

Simple C++ based 2D path planner for mobile robots using Particle Swarm Optimization algorithm.

Evolutionary algorithms are Meta-heuristic search or optimization methods suitable in simpler areas where a Learning based model would be an over-kill. They provide an interesting alternative to conventional methods of path planning. These are robust with respect to noisy evaluation functions, and the handling of evaluation functions which do not yield a sensible result in given period of time is straightforward. The algorithms can easily be adjusted to the problem at hand. Almost any aspect of the algorithm may be changed and customized.

An important feature of PSO is that each particle’s movement is influenced by its local best known position but is also guided towards the best known positions in the search-space to accelerate the convergence. This makes the convergence very fast and efficient in tricky environments.

### Instructions::

* Setup gcc c++ compiler in the local machine. 
* On Command Prompt/Terminal, run the following command from the current directory where PSO.cpp is present:: gcc PSO.cpp -o PSO -lstdc++ -lm
* On successful compilation, an executable file with the name "PSO" will be generated. Run "./PSO" in the command prompt/terminal
   to launch the program.
* A csv file named "graph_data" will be created. The data will be saved in it.
* Run the python file "plot_csv.py" to visualize the result.

Currently the start and goal positions are not taken from the command line, so we need to change the values in the code itself, along with all the values of the parameters

### Obstacle Avoidance::

For obstacle avoidance, we have used relocation method instead of penalty method for obstacle avoidance. In penalty method, we remove the particles falling inside an obstacle, or the path connecting new and old point passes through an obstacle. However, in relocation method, particles’ positions are relocated, so no particles are lost.
The algorithm part for relocation of particles’ positions is:

![obs2](https://user-images.githubusercontent.com/38212000/64594766-41130500-d3ce-11e9-9dee-81d7a093f8aa.jpg)


### Parameter values::
A 2D square search space of 3000*3000-unit boundary is chosen. A wide range of parameters were used on a trial and error basis to determine the set of parameters giving the best results in terms of faster convergence, efficient obstacle avoidance, feasible path generation, avoiding local minima, preventing high overshoots, etc. The parameters that balances a trade-off between all these criteria are the following:

*	Starting coordinates taken: (1000, 1000)
*	Goal coordinates taken: (-500, -1000)
*	Number of obstacles taken: 7-10
*	Swarm size: 100
*	Number of max iterations: 200
*	Inertial weight coefficients, wmin: 0.2, wmax: 0.7
*	Local and Social coefficients, c1: 2, c2: 2
*	Max allowed Velocity: width / 10
*	Threshold to reach near goal to succeed: width/20
*	Threshold to avoid near obstacles: radius of obstacle * 0.1

### Results::
Console printed values shows the decreasing fitness value as the goal is approached. The best global coordinate for each iteration is also displayed.

Case 1                     |  Case 2
:-------------------------:|:-------------------------:
<img align="left" width="400" height="350" src="https://user-images.githubusercontent.com/38212000/64595106-e9c16480-d3ce-11e9-8e97-2253d5db6327.JPG"> |  ![7](https://user-images.githubusercontent.com/38212000/64595092-e4fcb080-d3ce-11e9-8d64-c19f83f4cb36.JPG)



### Graphical results::
Following graphs were plotted in python using the data gathered from the above.
   * Initial point: The point at the top right
   * Final point: The point at the bottom left
   * Obstacles: The red circles represent the obstacles in the area under consideration
   * Final path: The blue line represents the final path from initial to final point.
   * Color coding of particles: The colors of the particles are based on the iterations. With each iteration, the particles change color      from blue (initial particles distribution) to red (final particles distribution)

Points to note:
   * As per the algorithm, none of the particles are placed inside the boundaries of the obstacles.
   * Initial particles (blue in color) are randomly placed, thus are more scattered. The final particles (red in color) converge near        the goal point.
   * The algorithm finds out the least cost path even when the obstacles are very closely placed, thus proving efficient in such              scenarios.

Case 1                     |  Case 2
:-------------------------:|:-------------------------:
<img align="left" width="500" height="400" src="https://user-images.githubusercontent.com/38212000/64595792-50934d80-d3d0-11e9-93df-c17874818e06.JPG"> | <img align="left" width="500" height="400" src="https://user-images.githubusercontent.com/38212000/64595805-538e3e00-d3d0-11e9-9602-0517b9f7aab4.JPG">

Case 3                     |  Case 4
:-------------------------:|:-------------------------:
<img align="left" width="500" height="400" src="https://user-images.githubusercontent.com/38212000/64595812-56892e80-d3d0-11e9-9d26-eb7b5925bdac.JPG"> | <img align="left" width="500" height="400" src="https://user-images.githubusercontent.com/38212000/64595848-6a349500-d3d0-11e9-8a46-023a851efd78.JPG">

### Future Work
* Changes are required to make all parameter inputs from command line arguments
* Fitness function dependencies needs modification to make the algorithm generic, so that it can be used in any optimisation problem, not just for path planning.
* Obstacles of different shapes and sizes, and also dynamic obstacles needs to be implemented. Dependency on the obstacle radius is to be removed as that information will not be available in practical cases. Instead we need to use the circumference information.
* B-spline curve is to be implemented to make the orientation changes smoother for a non-holonomic robot, at each best particle position. The algorithm should decide on the best coefficients for the B-spline curves.

