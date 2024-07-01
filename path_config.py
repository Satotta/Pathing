import numpy as np
from obstacles import Rectangle, Circle, Dropoff, Pickup
from pathing import findPath


# Create a list of obstacles
Rectangle(np.array([6, 10]), 0.125, 2, 2)

Rectangle(np.array([5, 2]), 0.125, 1.5, 1.5)

Circle(np.array([2, 4]), 0.25, 1.75)

Circle(np.array([11, 6]), 0.25, 0.5)

Rectangle(np.array([3, 10]), 0.125, 2, 2)

Rectangle(np.array([6, 6]), 0.125, 2, 3)

Circle(np.array([0.5, 8]), 0.125, 0.5)


# Create a list of dropoffs
Dropoff(1, 11)

Dropoff(12, 9)

Dropoff(11, 3)

Dropoff(8, 10)

Dropoff(4.5, 10)

Dropoff(10.5, 5)


# Create pickup
Pickup(4.4, 0.9)


# Call findPath method
path = findPath()

