# Pathing

The following files are used in conjunction to find the optimal path from a pickup, to every dropoff, and then back to the pickup while avoiding static obstacles (Circular/Rectangular).

To run, open the 'path_config.py' and add as many obstacles and dropoffs as desired and define a pickup location. Note that the algorithm uses permutations of the dropoffs to determine the optimal route. This means that more than 10 dropoffs may be too computationally intensive for most home computers.

File 'obstacles.py' contains the classes and class-specific methods (OOP). File 'pathing.py' contains the algorithm to loop through and call the class-specific methods as necessary to generate the optimal path.

See code comments for further details.
