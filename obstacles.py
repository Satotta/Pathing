import numpy as np
import math
from scipy.spatial.distance import cdist


# Define abstacle class
class Obstacle:


    # Init total number of obstacles
    number_of_obstacles = 0

    # Init list of centers
    centers = []

    # Init list of obstacles
    obstacles = []



    # Define init method for obstacle
    def __init__(self, center, offset):

        # Define new center
        self.center = center

        # Define new offset
        self.offset = offset

        # Append new obstacle to list
        Obstacle.obstacles.append(self)
        
        # Determine obstacle count
        Obstacle.number_of_obstacles += 1

        # Append new center to list
        if Obstacle.number_of_obstacles == 1:
            
            Obstacle.centers = self.center

        else:

            Obstacle.centers = np.vstack((Obstacle.centers, self.center))

    

    # Define plot method to plot every obstacle
    def plot(plt, ax):
        
        # Loop through each obstacle
        for ob in Obstacle.obstacles:
            
            # Plot obstacle depending on type
            if isinstance(ob, Circle):
                circle = plt.Circle((ob.center[0], ob.center[1]), ob.true_radius, color = 'c')
                ax.add_patch(circle)

            else:
                anchor_x = ob.center[0] - ob.width / 2
                anchor_y = ob.center[1] - ob.height / 2
                rectangle = plt.Rectangle((anchor_x, anchor_y), ob.width, ob.height, color = 'b')
                ax.add_patch(rectangle)



    # Define the sort method to sort obstacles based on distance from a given point
    def sort(path_point):

        # Init obstacles_sorted list
        obstacles_sorted = Obstacle.obstacles

        # If more than one obstacle 
        if Obstacle.number_of_obstacles > 1:

            # Create dropoff matrix the same size as obstacles 
            start_pose_matrix = path_point
            for i in range(1, Obstacle.number_of_obstacles):
                start_pose_matrix = np.vstack((start_pose_matrix, path_point))
                        
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

        return obstacles_sorted






# Define the circle class
class Circle(Obstacle):



    # Define the init method
    def __init__(self, center, offset, radius):

        # Inheret center and offset from Obstacle class
        super().__init__(center, offset)

        # Define new effective radius
        self.radius = radius + offset

        # Define new true radius
        self.true_radius = radius



    # Define tangent method to determine tangents along circumfrence
    def tangent(self, p):
        
        # Find distance between point and obstacle center
        dist = math.dist(p, self.center)
        
        # Find difference vector
        vec = p - self.center

        # Determine slope angle of difference vector
        theta = math.atan2(vec[1], vec[0])
        
        # Determine angle between diff vec and radius
        alpha = math.acos(self.radius / dist)

        # Define first tangent point
        t1 = np.array([self.center[0] + self.radius * math.cos(theta + alpha), 
                       self.center[1] + self.radius * math.sin(theta + alpha)])
        
        # Define second tangent point
        t2 = np.array([self.center[0] + self.radius * math.cos(theta - alpha), 
                       self.center[1] + self.radius * math.sin(theta - alpha)])
        
        return t1, t2
        


    # Define the do_intersect method to determine if segment intersects with obstacle
    def do_intersect(self, start_pose, end_pose):

        # Init closest point (point on segment that is closest to the circle center)
        closest = []

        # If segment vertical, then closest point is x-pos of the segment and y-pos of the center
        if end_pose[0] == start_pose[0]:

            closest = np.array([end_pose[0], self.center[1]])

        # If segment horizontal, then closest point is y-pos of the segment and x-pos of the center
        elif end_pose[1] == start_pose[1]:

            closest = np.array([self.center[0], end_pose[1]])

        # Otherwise, slope of segment is defined
        else:
            
            # Determine difference vector
            diff = end_pose - start_pose

            # Determine slope of segement
            m = diff[1] / diff[0]

            # Find x-pos of closest point
            x_closest = (m * start_pose[0] + self.center[0] / m + self.center[1] - start_pose[1]) / (m + 1 / m)

            # Find y-pos of closest point
            y_closest = m * (x_closest - start_pose[0]) + start_pose[1]

            # Define closest point
            closest = np.array([x_closest, y_closest])

        # Find distance between center and closest point
        dist = round(math.dist(closest, self.center), 5)

        #Define bounds to ensure closest point is within the segment bounds
        bounds_x = np.array([start_pose[0], end_pose[0]])
        bounds_y = np.array([start_pose[1], end_pose[1]])
        bounds_x = np.sort(bounds_x)
        bounds_y = np.sort(bounds_y)
        
        # Define if x-pos of closest point is within bounds
        btw_x = closest[0] >= bounds_x[0] and closest[0] <= bounds_x[1]

        # Define if y-pos of closest point is within bounds
        btw_y = closest[1] >= bounds_y[0] and closest[1] <= bounds_y[1]

        # If within bounds and distance less than radius
        if dist < self.radius and btw_x and btw_y:
            
            # Return true, intersection occurs
            return True
        
        # Return false, no intersection occurs
        return False
    
    

    # Define avoid method to determine intermediate point between start and ended needed to avoid obstacle
    def avoid(self, start_pose, end_pose):

        # Find tangent points for start pose
        t1_do1, t2_do1 = self.tangent(start_pose)

        # Find tangent points for end pose
        t1_do2, t2_do2 = self.tangent(end_pose)

        # Define list of tangents
        points = [t1_do1, t2_do2, t2_do1, t1_do2]

        # Init path segment distance as high value
        dist = np.inf

        # Init necessary intermediate point
        path_point = []

        # Loop through tangents
        for i in range(0, 3, 2):
            
            # Find possible intermediate point for this iteration
            int = self.line_intersect(start_pose, points[i], end_pose, points[i + 1])

            # Determine total path segement for this iteration
            dist_temp = math.dist(start_pose, int) + math.dist(end_pose, int)

            # If this iteration path distance is less than previous
            if dist_temp < dist:
                
                # Set previous path distance to current
                dist = dist_temp

                # Set intermediat point to this iterations
                path_point = int
        
        # Return the new rerouted path
        return np.vstack((path_point, end_pose))
            


    # Define the line_intersect method to determine the intersection point between two lines
    def line_intersect(self, p1, p2, q1, q2):
        
        # Init x intersection point
        x_int = 0

        # Init y intersection point
        y_int = 0
        
        # If line p is vertical
        if p1[0] == p2[0]:
            
            # Find slope of line q
            m_q = (q2[1] - q1[1]) / (q2[0] - q1[0])
            
            # x intersection point is x-pos of line p
            x_int = p1[0]

            # Find y-pos of intersection from x-pos and equation of line q
            y_int = m_q * (x_int - q1[0]) + q1[1]

            # Return intersection point
            return np.array([x_int, y_int])
        
        # If line q is vertical 
        if q1[0] == q2[0]:
            
            # Find slope of line p
            m_p = (p2[1] - p1[1]) / (p2[0] - p1[0])

            # x intersection point is x-pos of line q
            x_int = q1[0]

            # Find y-pos of intersection from x-pos and equation of line p
            y_int = m_p * (x_int - p1[0]) + p1[1]

            # Return intersection point
            return np.array([x_int, y_int])

        # If line p is horizontal
        if p1[1] == p2[1]:
            
            # Find slope of line q
            m_q = (q2[1] - q1[1]) / (q2[0] - q1[0])

            # y intersection point is y-pos of line p
            y_int = p1[1]

            # Find x-pos of intersection from y-pos and equation of line q
            x_int = (y_int - q1[1]) / m_q + q1[0]

            # Return intersection point
            return np.array([x_int, y_int])

        # If line q is horizontal
        if q1[1] == q2[1]:
            
            # Find slope of line p
            m_p = (p2[1] - p1[1]) / (p2[0] - p1[0])

            # y intersection point is y-pos of line q
            y_int = q1[1]

            # Find x-pos of intersection from y-pos and equation of line p
            x_int = (y_int - p1[1]) / m_p + p1[0]

            # Return intersection point
            return np.array([x_int, y_int])

        # Otherwise, slope of both lines is defined and non-zero
        m_p = (p2[1] - p1[1]) / (p2[0] - p1[0])
        m_q = (q2[1] - q1[1]) / (q2[0] - q1[0])

        # Find x intersection by equating p and q
        x_int = (m_p * p1[0] + q1[1] - m_q * q1[0] - p1[1]) / (m_p - m_q)
        
        # Find y from x-pos and equation of line p
        y_int = m_p * (x_int - p1[0]) + p1[1]

        # Return intersection point
        return np.array([x_int, y_int])
    


    # Define pathing method to determine necessary path to get from start_pose to end_pose
    def pathing(self, start_pose, end_pose):

        if self.do_intersect(start_pose, end_pose):
            
            return self.avoid(start_pose, end_pose)

        return end_pose






# Define the rectangle class
class Rectangle(Obstacle):

    # Init the segments variable
    segments = []



    # Define the init method
    def __init__(self, center, offset, width, height):
        
        # Inheret center and offset from Obstacle class
        super().__init__(center, offset)

        # Define new width
        self.width = width

        # Define new height
        self.height = height   
    

    
    # Define the get_vertices method to find all vertices of rectangle
    def get_vertices(self):
        
        # Define x-pos and y-pos of center
        x = self.center[0]
        y = self.center[1]

        # Define all x-pos and y-pos of vertices (include offset)
        x_left = x - self.width/2 - self.offset
        x_right = x + self.width/2 + self.offset
        y_bot = y - self.height/2 - self.offset
        y_top = y + self.height/2 + self.offset

        # Define the bottom left vertice
        self.bot_left = np.array([x_left, y_bot])

        # Define the top left vertice
        self.top_left = np.array([x_left, y_top])

        # Define the top right vertice
        self.top_right = np.array([x_right, y_top])

        # Define the bottom right vertice
        self.bot_right = np.array([x_right, y_bot])



    # Define the get_segments method to return the possible segments that can be intersected for a given start and end pose
    def get_segments(self, start_pose, end_pose):
        
        # Get rectangle vertices
        self.get_vertices()

        # Define obstacle segements from vertices
        left = np.vstack((self.bot_left, self.top_left))              
        right = np.vstack((self.bot_right, self.top_right))
        top = np.vstack((self.top_left, self.top_right))
        bot = np.vstack((self.bot_left, self.bot_right))
        
        # Determine direction of current path segment
        xdir = end_pose[0] - start_pose[0]
        ydir = end_pose[1] - start_pose[1]

        # If right and up
        if xdir >= 0 and ydir >= 0:
            
            # Return segments in correct order to be checked
            return np.vstack((left, bot, right, top))
        
        # If right and down
        elif xdir >= 0 and ydir <= 0:
            
            # Return segments in correct order to be checked
            return np.vstack((left, top, right, bot))

        # If left and up
        elif xdir <= 0 and ydir >= 0:
            
            # Return segments in correct order to be checked
            return np.vstack((right, bot, left, top))

        # If left and down
        else:

            # Return segments in correct order to be checked
            return np.vstack((right, top, left, bot))
    


    # Define orientation method to determine the orientation of a point with resepct to a line segment
    def orientation(self, p, q, r):
        
        # Define orientation value
        s = (q[1] - p[1])*(r[0] - q[0]) - (q[0] - p[0])*(r[1] - q[1])

        # If 0, point is colinear to segment
        if s == 0:

            return 0
        
        # If positive, point is clockwise to segment
        elif s > 0:

            return 1
        
        # If negative, point is counter-clockwise to segment
        else:

            return 2
        
        

    # Define on_segment method to determine if a point is on a line segement
    def on_segment(self, p, q, r):
        
        # If point within segement bounds
        if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
            
            return True
        
        return False



    # Define the do_intersect method to determine if the current path segment intersects with the current edge being checked
    def do_intersect(self, start_pose, end_pose, start_edge, end_edge):
        
        # Define possible orientations for the given points
        o1 = self.orientation(start_pose, end_pose, start_edge)
        o2 = self.orientation(start_pose, end_pose, end_edge)
        o3 = self.orientation(start_edge, end_edge, start_pose)
        o4 = self.orientation(start_edge, end_edge, end_pose)

        # If line is colinear, no need to reroute
        if (o1 == 0) and (o2 == 0):

            return False
        
        # Lines intersect
        if (o1 != o2) and (o3 != o4):
                
            return True
        
        # Special cases for when a point from one of the lines lands on the other segment
        if ((o1 == 0) and self.on_segment(start_pose, start_edge, end_pose)):

            return True
            
        if ((o2 == 0) and self.on_segment(start_pose, end_edge, end_pose)):

            return True
            
        if ((o3 == 0) and self.on_segment(start_edge, start_pose, end_edge)):

            return True
            
        if ((o4 == 0) and self.on_segment(start_edge, end_pose, end_edge)):

            return True
        
        # Otherwise, no intersection
        return False



    # Define on_vertices method to determine if a point lies on any of the vertices
    def on_vertices(self, path_point):

        # Get the vertices
        self.get_vertices()

        # Return true if point lies on any of the vertices
        if np.array_equal(path_point, self.bot_left):

            return True

        if np.array_equal(path_point, self.top_left):

            return True

        if np.array_equal(path_point, self.top_right):

            return True

        if np.array_equal(path_point, self.bot_right):

            return True

        # Otherwise return false
        return False



    # Define pathing method to determine necessary intermediate points to avoid obstacle
    def pathing(self, start_pose, end_pose):
        
        # Get the possible segements that can be intersected
        Rectangle.segments = self.get_segments(start_pose, end_pose)

        # Loop through segements
        for i in range(0, 3, 2):
            
            # Define the Left and Right or Top and Bottom edges to be checked (depends on iteration)
            first_start_edge = Rectangle.segments[i]
            first_end_edge = Rectangle.segments[i + 1]
            second_start_edge = Rectangle.segments[i + 4]
            second_end_edge = Rectangle.segments[i + 5]

            # Determine if the first and second edges are intersected by current segment
            intersect_first = self.do_intersect(start_pose, end_pose, first_start_edge, first_end_edge)
            intersect_second = self.do_intersect(start_pose, end_pose, second_start_edge, second_end_edge)

            # If both intersect, two intermediate points may be necessary
            if intersect_first and intersect_second:   
                
                # Determine distances from start pose to first edge vertices
                dist1 = math.dist(first_start_edge, end_pose)
                dist2 = math.dist(first_end_edge, end_pose)

                # Define correct index value
                j = 0
                if i == 0:

                    j = 1
                
                # If start point of edge is closer, reroute to it and
                if dist1 < dist2:
                    
                    # If end_pose is within bounds of the start point of edge
                    if end_pose[j] < self.top_right[j] and end_pose[j] > self.bot_left[j]:
                        
                        # Return two rerouting points
                        return np.vstack((first_start_edge, second_start_edge, end_pose))
                    
                    # Otherwise return one
                    return np.vstack((first_start_edge, end_pose))

                # Otherwise if end_pose is within bounds of the end point of edge
                if end_pose[j] < self.top_right[j] and end_pose[j] > self.bot_left[j]:
                        
                        # Return it and second point
                        return np.vstack((first_end_edge, second_end_edge, end_pose))

                # Otherwise return just it
                return np.vstack((first_end_edge, end_pose))   

            # If only intersects first edge
            elif intersect_first:
                
                # Determine distances from end_pose to edge vertices
                dist1 = math.dist(first_start_edge, end_pose)
                dist2 = math.dist(first_end_edge, end_pose)

                # If start point of edge is closer, reroute to it
                if dist1 < dist2:
                    
                    return np.vstack((first_start_edge, end_pose))
                
                # Otherwise reroute to end point of edge
                return np.vstack((first_end_edge, end_pose))
        
        # If no intersection, do not reroute
        return end_pose






# Define the Dropoff class
class Dropoff:



    # Init the number of dropoffs
    number_of_dropoffs = 0
    
    # Init the list of dropoffs
    dropoffs = []
    

    # Define the init method
    def __init__(self, x, y):
        
        # Define the new dropoff location
        self.dropoff = np.array([[x, y]])

        # Add to the total number of dropoffs
        Dropoff.number_of_dropoffs += 1

        # Stack dropoff locations accordingly
        if Dropoff.number_of_dropoffs == 1:

            Dropoff.dropoffs = self.dropoff

        else:

            Dropoff.dropoffs = np.vstack((Dropoff.dropoffs, self.dropoff))



    # Define plot method to plot the dropoffs
    def plot(ax):

        # Loop through dropoffs
        for i in range(Dropoff.number_of_dropoffs):
            
            # Plot each dropoff on given axes
            ax.plot(Dropoff.dropoffs[i, 0], Dropoff.dropoffs[i, 1], marker = '^', color = 'r')






# Define the Pickup class
class Pickup:

    # Init the location variable
    location = []


    # Define the init method
    def __init__(self, x, y):
        
        # Apply the global pickup location (will be overwritten if two pickups are defined)
        Pickup.location = np.array([[x, y]])



    # Define plot method to plot the pickup
    def plot(ax):

        # Plot the pickup on given axes
        ax.plot(Pickup.location[0, 0], Pickup.location[0, 1], marker = 'o', color = 'g')





