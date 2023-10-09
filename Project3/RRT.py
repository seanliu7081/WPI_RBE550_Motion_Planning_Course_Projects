# Standard Algorithm Implementation
# Sampling-based Algorithms RRT

import matplotlib.pyplot as plt
import numpy as np


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.start = Node(start[0], start[1])  # start node
        self.goal = Node(goal[0], goal[1])  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag

    def init_map(self):
        """Intialize the map before each search"""
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    # def dis(self, node1, node2):
    #     """Calculate the euclidean distance between two nodes
    #     arguments:
    #         node1 - node 1
    #         node2 - node 2

    #     return:
    #         euclidean distance between two nodes
    #     """
    #     # ### YOUR CODE HERE ###
    #     return 0

    def dis(self, node1, node2):
    # Extract the row and col values
        row1, col1 = node1.row, node1.col
        row2, col2 = node2.row, node2.col
    
    # Calculate the squared differences
        d_row = row1 - row2
        d_col = col1 - col2
    
    # Return the euclidean distance
        return np.sqrt(d_row**2 + d_col**2)


    # def check_collision(self, node1, node2):
    #     """Check if the path between two nodes collide with obstacles
    #     arguments:
    #         node1 - node 1
    #         node2 - node 2

    #     return:
    #         True if the new node is valid to be connected
    #     """
    #     # ### YOUR CODE HERE ###
    #     return True

    def check_collision(self, node1, node2):

    # Extract the row and col values
        x1, y1 = node1.row, node1.col
        x2, y2 = node2.row, node2.col

    # Use Bresenham's line algorithm to get points on the line
        points = self.bresenham(x1, y1, x2, y2)

    # Check if any point on the line is an obstacle
        for (x, y) in points:
            x, y = int(x), int(y)
            if self.map_array[x, y] == 0:  # Assuming 0 indicates obstacle in map_array
                return False

        return True

    def bresenham(self, x1, y1, x2, y2):

        points = []
        is_steep = abs(y2 - y1) > abs(x2 - x1)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1
        dy = y2 - y1
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
        y = y1

        x1, x2 = int(x1), int(x2)

        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points


    # def get_new_point(self, goal_bias):
    #     """Choose the goal or generate a random point
    #     arguments:
    #         goal_bias - the possibility of choosing the goal
    #                     instead of a random point

    #     return:
    #         point - the new point
    #     """
    #     # ### YOUR CODE HERE ###
    #     return self.goal

    def get_new_point(self, goal_bias):

    # If a random float between 0 and 1 is less than goal_bias, return the goal
        if np.random.rand() < goal_bias:
            return [self.goal.row, self.goal.col]
    
    # Otherwise, return a random point within the map boundaries
        random_row = np.random.randint(0, self.size_row)
        random_col = np.random.randint(0, self.size_col)
    
        return [random_row, random_col]


    # def get_nearest_node(self, point):
    #     """Find the nearest node in self.vertices with respect to the new point
    #     arguments:
    #         point - the new point

    #     return:
    #         the nearest node
    #     """
    #     # ### YOUR CODE HERE ###
    #     return self.vertices[0]

    def get_nearest_node(self, point):
        min_distance = float('inf')  # initialize with a high value
        nearest_node = None

        for node in self.vertices:
            distance = np.linalg.norm([node.row - point[0], node.col - point[1]])
            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node

    # def get_neighbors(self, new_node, neighbor_size):
    #     """Get the neighbors that are within the neighbor distance from the node
    #     arguments:
    #         new_node - a new node
    #         neighbor_size - the neighbor distance

    #     return:
    #         neighbors - list of neighbors that are within the neighbor distance
    #     """
    #     # ### YOUR CODE HERE ###
    #     return [self.vertices[0]]

    def get_neighbors(self, new_node, neighbor_size):
        neighbors = []

        for node in self.vertices:
            distance = np.linalg.norm([node.row - new_node.row, node.col - new_node.col])
            if distance < neighbor_size:
                neighbors.append(node)

        return neighbors

    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker="o", color="y")
            plt.plot(
                [node.col, node.parent.col],
                [node.row, node.parent.row],
                color="y",
            )

        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot(
                    [cur.col, cur.parent.col],
                    [cur.row, cur.parent.row],
                    color="b",
                )
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker="o", color="b")

        # Draw start and goal
        plt.plot(
            self.start.col, self.start.row, markersize=5, marker="o", color="g"
        )
        plt.plot(
            self.goal.col, self.goal.row, markersize=5, marker="o", color="r"
        )

        # show image
        plt.show()

    # def RRT(self, n_pts=1000):
    #     """RRT main search function
    #     arguments:
    #         n_pts - number of points try to sample,
    #                 not the number of final sampled points

    #     In each step, extend a new node if possible,
    #     and check if reached the goal
    #     """
    #     # Remove previous result
    #     self.init_map()

    #     # ### YOUR CODE HERE ###

    #     # In each step,
    #     # get a new point,
    #     # get its nearest node,
    #     # extend the node and check collision to decide whether to add or drop,
    #     # if added, check if reach the neighbor region of the goal.

    #     # Output
    #     if self.found:
    #         steps = len(self.vertices) - 2
    #         length = self.goal.cost
    #         print("It took %d nodes to find the current path" % steps)
    #         print("The path length is %.2f" % length)
    #     else:
    #         print("No path found")

    #     # Draw result
    #     self.draw_map()


    def RRT(self, n_pts=1000, extend_length=5.0, goal_region=5.0):

    # Remove previous result
        self.init_map()

        for _ in range(n_pts):
            if self.found:
                break

        # Get a new random point with a certain bias towards the goal
            point = self.get_new_point(0.1)  # for example, 10% bias towards goal

        # Find nearest node to the point
            nearest_node = self.get_nearest_node(point)

        # Get direction to the new point from the nearest node
            theta = np.arctan2(point[1] - nearest_node.col, point[0] - nearest_node.row)

        # Extend the nearest node towards the new point
            new_point = [nearest_node.row + extend_length * np.cos(theta),
                     nearest_node.col + extend_length * np.sin(theta)]

            new_node = Node(new_point[0], new_point[1])
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + extend_length

        # Check if the path to the new node is collision free
            if not self.check_collision(nearest_node, new_node):
                continue  # if collision, discard this point and move on

        # If no collision, add the new node to the vertices
            self.vertices.append(new_node)

        # Check if we are close enough to the goal
            if self.dis(new_node, self.goal) <= goal_region:
                self.found = True
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.dis(new_node, self.goal)
                self.vertices.append(self.goal)

    # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

    # Draw result
        self.draw_map()
