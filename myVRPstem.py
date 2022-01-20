from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import numpy as np
import random
from scipy.spatial import distance_matrix

import math
import matplotlib.pyplot as plt
from mpl_toolkits.axisartist import SubplotZero

num_stops = 268
attempts = 200
vehicles = 13
hub_area = 1.19
vehicle_vec = list(range(1,15))

for j in vehicle_vec:
    vehicles = j
    print(vehicles)
    ###########################################################################################
    ##################################### For TSP: ############################################
    ###########################################################################################
    def create_data_model():
        """Stores the data for the problem."""
        data = {}

        # Create random points
        #print('Enter desired number of points:')
        #no_points = int(input())
        no_points = num_stops
        print('Registered points:', no_points)
        # Generate random points
        x = [[random.randint(0, 1000), random.randint(0, 1000)]
            for i in range(no_points)]
        matrix = distance_matrix(x, x)
        matrix = ((100 * matrix).round()).astype(int)
        data['distance_matrix'] = matrix
        #data['num_vehicles'] = 5
        data['depot'] = 0
        return data


    def print_solution(manager, routing, solution):
        """Prints solution on console."""
        print('Objective: {} miles'.format(solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route for vehicle 0:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        plan_output += 'Route distance: {}miles\n'.format(route_distance)


    ###########################################################################################
    ##################################### For plotting: #######################################
    ###########################################################################################
    class Axes():

        def __init__(self, xlim=(-5, 5), ylim=(-5, 5), figsize=(12, 5)):
            self.xlim = xlim
            self.ylim = ylim
            self.figsize = figsize
            self.points = []
            self.segments = []
            self.vectors = []
            self.lines = []
            self.scale_arrows()

        def __arrow__(self, x, y, dx, dy, width, length):
            plt.arrow(
                x, y, dx, dy,
                color='k',
                clip_on=False,
                head_width=self.head_width,
                head_length=self.head_length
            )

        def __drawAxis__(self):
            """
            Draws the 2D cartesian axis
            """
            # A subplot with two additional axis, "xzero" and "yzero"
            # corresponding to the cartesian axis
            ax = SubplotZero(self.fig, 1, 1, 1)
            self.fig.add_subplot(ax)

            # make xzero axis (horizontal axis line through y=0) visible.
            for axis in ["xzero", "yzero"]:
                ax.axis[axis].set_visible(True)
            # make the other axis (left, bottom, top, right) invisible
            for n in ["left", "right", "bottom", "top"]:
                ax.axis[n].set_visible(False)

            # Plot limits
            plt.xlim(self.xlim)
            plt.ylim(self.ylim)
            # Draw the arrows
            self.__arrow__(self.xlim[1], 0, 0.01, 0, 0.3, 0.2)  # x-axis arrow
            self.__arrow__(0, self.ylim[1], 0, 0.01, 0.2, 0.3)  # y-axis arrow

        def scale_arrows(self):
            """ Make the arrows look good regardless of the axis limits """
            xrange = self.xlim[1] - self.xlim[0]
            yrange = self.ylim[1] - self.ylim[0]

            self.head_width = min(xrange / 30, 0.25)
            self.head_length = min(yrange / 30, 0.3)

        def draw(self, image=None):
            self.scale_arrows()
            self.fig = plt.figure(figsize=self.figsize)
            # First draw the axis
            self.__drawAxis__()
            # Plot each point
            for point in self.points:
                point.draw()
            # Save the image?
            if image:
                plt.savefig(image)
            plt.show()

        def addPoints(self, points):
            for p in points:
                self.addPoint(p)

        def addPoint(self, p):
            self.points.append(p)


    class Point():

        def __init__(self, x, y, color='#4ca3dd', size=20, add_coordinates=False):
            self.x = x
            self.y = y
            self.color = color
            self.size = size
            self.add_coordinates = add_coordinates
            self.y_offset = 0.2
            self.items = np.array([x, y])
            self.len = 2

        def __getitem__(self, index):
            return self.items[index]

        def __str__(self):
            return "Point(%.2f,%.2f)" % (self.x, self.y)

        def __repr__(self):
            return "Point(%.2f,%.2f)" % (self.x, self.y)

        def __len__(self):
            return self.len

        def draw(self):
            plt.scatter([self.x], [self.y], color=self.color, s=self.size)

            # Add the coordinates if asked by user
            if self.add_coordinates:
                plt.text(
                    self.x, self.y + self.y_offset,
                    "(%.1f,%.1f)" % (self.x, self.y),
                    horizontalalignment='center',
                    verticalalignment='bottom',
                    fontsize=8
                )

    ###########################################################################################
    ##################################### Main: ###############################################
    ###########################################################################################


    def main():
        """Entry point of the program."""
        # Create random points
        
        no_points = num_stops
        # Add one point to account for "dummy" depot
        no_points += 1

        iterations = attempts

        print()
        print('Performing calculation..')

        avg_length = 0

        for i in range(iterations):
            # Generate no_points random points
            x = [[random.randint(0, 1000), random.randint(0, 1000)]
                for i in range(no_points)]

            x[0]= [500,500] #set hub centrally
            matrix = distance_matrix(x, x)
            matrix = ((100 * matrix).round()).astype(int)

            # Visualise random points on coordinate system:
            # Create the cartesian axis
            axes = Axes(xlim=(-1000, 2000), ylim=(-1000, 2000), figsize=(9, 7))
            
            # Create two points
            points = []
            for i in range(1, no_points):
                points.append(Point(x[i][0],  x[i][1], color='#ffa500'))
            axes.addPoints(points)
            # axes.draw()

            # Initialise data structure
            """Stores the data for the problem."""
            data = {}
            data['distance_matrix'] = matrix
            data['num_vehicles'] = vehicles
            data['depot'] = 0

            # Create the routing index manager
            manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                                data['num_vehicles'], data['depot'])

            # Create Routing Model.
            routing = pywrapcp.RoutingModel(manager)

            def distance_callback(from_index, to_index):
                """Returns the distance between the two nodes."""
                # Convert from routing variable Index to distance matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return data['distance_matrix'][from_node][to_node]

            transit_callback_index = routing.RegisterTransitCallback(
                distance_callback)

            # Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            # Setting first solution heuristic.
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

            # Solve the problem.
            solution = routing.SolveWithParameters(search_parameters)

            # Print solution on console.
            if solution:
                # print_solution(manager, routing, solution)
                avg_length += solution.ObjectiveValue()

        print('Average length of total path for each vehicle: ', avg_length / (iterations*100000), 'km.')
    

    if __name__ == '__main__':
        main()