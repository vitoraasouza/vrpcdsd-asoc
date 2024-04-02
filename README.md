# Optimization solvers for the Vehicle Routing Problem with Cross-Docking and Scheduling at the Docking Station (VRPCDSD)

This repository has the source codes of two methods designed to solve the VRPCDSD. One is the General Variable Neighborhood Search (GVNS) metaheuristic, and the other is a Mixed Integer Program (MIP) formulation, that is solved by using the CPLEX's Branch-and-Cut.

Besides, the instances used to evaluate both methods are available at _problem\_instances_. Don't forget to check its readme file to understand the naming correspondences between the instances in this repo and the ones in the paper.

Lastly, in folder _utils_, there is a little program which is used to draw the solutions obtained by GVNS and MIP in LaTeX, using TikZ.
