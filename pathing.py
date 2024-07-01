import numpy as np
from scipy.spatial.distance import cdist
from itertools import permutations
import math
from obstacles import Obstacle, Rectangle, Dropoff, Pickup
import matplotlib.pyplot as plt
import time

def findPath():

    # Establish start time
    t0 = time.time()

    # Find all permutation possibilities
    drop_perms = np.array(list(permutations(range(0, Dropoff.number_of_dropoffs))))

    # Initialize shortest distance
    dist_prev = np.inf

    # Initialize shortest path
    path = []

    # Loop through every permutation possibility
    for p in range(len(drop_perms)):

        # Find dropoff order for this iteration
        path_index = drop_perms[p, :]

        # Initialize the path for this iteration
        path_temp = Pickup.location

        # Initialize rerouting point(s) counter
        c = 0

        for i in range(1, Dropoff.number_of_dropoffs + 2):

            # Initialize start_pose
            start_pose = path_temp[i - 1 + c, :]
            end_pose = []

            # Determine correct end pose depending on iteration
            if i == Dropoff.number_of_dropoffs + 1:

                end_pose = Pickup.location[0, :]

            else:

                end_pose = Dropoff.dropoffs[path_index[i - 1], :]

            # Set recursive flag
            flag = True

            # For current start and end, determine path until no intersections occur
            while flag:
                
                # Init not_hit flag to determine whether this iteration hit an object
                not_hit = True
 
                # Sort the obstacles based on distance from start_pose
                obstacles_sorted = Obstacle.sort(start_pose)

                # Sort through each obstacle and search for intersection
                for ob in obstacles_sorted:

                    # Check to see if object is a rectangle
                    if isinstance(ob, Rectangle):

                        # Check if start_pose is on any of the vertices
                        if ob.on_vertices(start_pose):
                            
                            # If so, this object has already been avoided so skip this object
                            continue

                    # Determine intersection
                    new_path = ob.pathing(start_pose, end_pose)
                    
                    # Find necessary count indexing for path size
                    size_index = int(new_path.size / 2 - 1)
                    
                    # If larger than zero than a rerouting poin tis necessary
                    if size_index > 0:
                        
                        # Define the updated start pose as last rerouting point in new path
                        start_pose = new_path[size_index - 1, :]

                        # Append the rerouting point(s) to the current path
                        path_temp = np.vstack((path_temp, new_path[0:size_index, :]))

                        # Update the not_hit flag to False as an obstacle was hit
                        not_hit = False

                        # Update rerouting point(s) counter
                        c += size_index

                        # Break obstacle loop as an intersection was detected
                        break
                
                # Determine if an obstacle was hit
                if not_hit:

                    # Set flag to false as there is no obstacle to avoid
                    flag = False

                    # Update path
                    path_temp = np.vstack((path_temp, end_pose))

        # Determine total length of current path
        dist_temp = -1
        for i in range(1, len(path_temp)):
            
            # Determine distance of current segement
            dist_segment = math.dist(path_temp[(i - 1), :], path_temp[i, :])

            # Add to total distance of current path
            dist_temp += dist_segment

        # Determine if current path is shorter than previous
        if dist_temp < dist_prev:

            # If so, store this path
            path = path_temp

            # Set prev path to new shartest
            dist_prev = dist_temp
    
    # Define figure for plot
    fig = plt.figure(1)

    # Define axes
    ax = fig.add_subplot()

    # Plot the local path
    ax.plot(path[:, 0], path[:, 1], color = 'k', linestyle = ':')
    
    # Plot pickup
    Pickup.plot(ax)

    # Plot each dropoff
    Dropoff.plot(ax)
    
    # Plot each obstacle
    Obstacle.plot(plt, ax)
    
    # Define plot properties
    ax.grid(True)
    ax.set_title('Optimized Path Visualization')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(['Optimized Path', 'Pickup', 'Dropoffs'])

    # Display how much time has elapsed
    tf = time.time()
    dt = round(tf - t0, 4)
    print(f'Time elapsed: {dt} seconds')
    
    # Show plot
    plt.show()
    
    # Return optimized path
    return path


    