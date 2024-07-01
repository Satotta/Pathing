import numpy as np
from scipy.spatial.distance import cdist
from itertools import permutations
import math
from obstacles import Obstacle, Rectangle, Circle, Dropoff, Pickup
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

        # Initialize counter for this iteration
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

                # Determine obstacle type
                if Obstacle.number_of_obstacles > 0:
                    
                    # Init obstacles_sorted list
                    obstacles_sorted = Obstacle.obstacles

                    # If more than one obstacle 
                    if Obstacle.number_of_obstacles > 1:

                        # Create dropoff matrix the same size as obstacles 
                        start_pose_matrix = start_pose
                        for j in range(1, Obstacle.number_of_obstacles):
                            start_pose_matrix = np.vstack((start_pose_matrix, start_pose))
                        
                        # Find distance between current start and each obstacle
                        ob_dists = []
                        ob_dists = cdist(Obstacle.centers, start_pose_matrix)
                        ob_dists = ob_dists[:, 0]
            
                        # Loop thorugh obstacles and sort from closest to farthest
                        min_index = np.argmin(ob_dists)
                        obstacles_sorted = np.array([Obstacle.obstacles[min_index]])
                        ob_dists[min_index] = np.inf

                        for j in range(1, Obstacle.number_of_obstacles):

                            # Find minimum distance index
                            min_index = np.argmin(ob_dists)

                            # Store obstacles order
                            obstacles_sorted = np.hstack((obstacles_sorted, Obstacle.obstacles[min_index]))

                            # Replace current distance value with large value
                            ob_dists[min_index] = np.inf
                    
                    # Define intersection counter for current path segment
                    k = 0

                    # Sort through each obstacle and search for intersection
                    for ob in obstacles_sorted:

                        # Check to see if object is a rectangle
                        if isinstance(ob, Rectangle):

                            # Check to see if starting position lies on a vertice
                            ob.get_vertices()

                            # If so, skip this object
                            if np.array_equal(start_pose, ob.bot_left):

                                continue

                            if np.array_equal(start_pose, ob.top_left):

                                continue

                            if np.array_equal(start_pose, ob.top_right):

                                continue

                            if np.array_equal(start_pose, ob.bot_right):

                                continue

                        # Determine intersection
                        new_path = ob.pathing(start_pose, end_pose)
                        
                        # Replace path with new path depending on number of rerouted points
                        if new_path.size == 4:
                            
                            # Set new start pose for next check
                            start_pose = new_path[0, :]

                            # Update path to reflect rerouting point
                            path_temp = np.vstack((path_temp, start_pose))

                            # Update counters now that rerouting point is necessary
                            c += 1
                            k += 1

                            # Break obstacle loop now that intersection has been found
                            break

                        if new_path.size == 6:
        
                            # Set new start pose for next check
                            start_pose = new_path[1, :]

                            # Update path to reflect rerouting point
                            path_temp = np.vstack((path_temp, new_path[0:2, :]))

                            # Update counters now that rerouting points are necessary
                            c += 2
                            k += 1

                            # Break obstacle loop now that intersection has been found
                            break
                    
                    # Determine if an obstacle was hit
                    if k == 0:

                        # Set flag to false as there is no obstacle to avoid
                        flag = False

                        # Update path
                        path_temp = np.vstack((path_temp, end_pose))
                
                # Otherwise if no obstacles
                else:

                    # Set flag to false as there are no obstacles
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
    
    # Plot the local path
    fig = plt.figure(1)
    ax1 = fig.add_subplot()
    ax1.plot(path[:, 0], path[:, 1], color = 'k', linestyle = ':')
    ax1.grid(True)
    ax1.set_title('Optimized Path Visualization')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')

    # Plot pickup
    ax1.plot(Pickup.location[0, 0], Pickup.location[0, 1], marker = 'o', color = 'g')

    # Plot each dropoff
    for i in range(Dropoff.number_of_dropoffs):
        
        ax1.plot(Dropoff.dropoffs[i, 0], Dropoff.dropoffs[i, 1], marker = '^', color = 'r')

    # Plot each obstacle
    for ob in Obstacle.obstacles:

        if isinstance(ob, Circle):
            circle = plt.Circle((ob.center[0], ob.center[1]), ob.true_radius, color = 'c')
            ax1.add_patch(circle)

        else:
            anchor_x = ob.center[0] - ob.width / 2
            anchor_y = ob.center[1] - ob.height / 2
            rectangle = plt.Rectangle((anchor_x, anchor_y), ob.width, ob.height, color = 'b')
            ax1.add_patch(rectangle)
    
    # Define legend
    ax1.legend(['Optimized Path', 'Pickup', 'Dropoffs'])

    # Display how much time has elapsed
    tf = time.time()
    dt = round(tf - t0, 4)
    print(f'Time elapsed: {dt} seconds')
    
    # Show plot
    plt.show()
    
    # Return optimized path
    return path


    
