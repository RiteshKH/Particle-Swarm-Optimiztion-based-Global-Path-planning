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
