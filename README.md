# Open House Routing
This is a vehicle routing model with time window (VRWTW). It can be used for the best route of visiting open houses. Given the list of address, the model will be able to compute a shortest route.

There are three major files:
- Data Preprocessing:
    - time_dist_data_process.py: Processing the address file, calling google map api to compute distance/time matrix for routing model.
- Routing model (either of the below):
	- get_best_route_pulp.py: Using [pulp solver](https://coin-or.github.io/pulp/) to solve the exact solution as mixed-integer programming model (MILP)
	- get_best_route_ortools.py: Calling [google ortool solver](https://developers.google.com/optimization/routing). Using its existing routing module to solve with heuristics approach. The heuristics algorithms include neighbourhood search, simulated annealing... 