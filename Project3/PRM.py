# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy.spatial import KDTree

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path
    
    def bresenham_line(self, p1, p2):
        points = []
        row1, col1 = p1
        row2, col2 = p2
        d_row = abs(row2 - row1)
        d_col = abs(col2 - col1)
        s_row = 1 if row1 < row2 else -1
        s_col = 1 if col1 < col2 else -1
        err = d_row - d_col

        while True:
            points.append((row1, col1))
            if row1 == row2 and col1 == col2:
                break
            e2 = 2 * err
            if e2 > -d_col:
                err -= d_col
                row1 += s_row
            if e2 < d_row:
                err += d_row
                col1 += s_col

        return points


    def check_collision(self, p1, p2):
        """Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles   two points
        """
        # ### YOUR CODE HERE ###
        for point in self.bresenham_line(p1, p2):
            row, col = point
            if 0 <= row < self.size_row and 0 <= col < self.size_col:
                if self.map_array[row][col] == 0:  # 0 denotes an obstacle
                    return True
        return False

    def dis(self, point1, point2):
        """Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        """
        # ### YOUR CODE HERE ###
        d_row = point1[0] - point2[0]
        d_col = point1[1] - point2[1]
        return np.sqrt(d_row*d_row + d_col*d_col)

    def uniform_sample(self, n_pts):
        """Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()
        self.samples = []

        # Sample and check for collisions
        for _ in range(n_pts):
            row = np.random.randint(0, self.size_row)
            col = np.random.randint(0, self.size_col)
            if self.map_array[row, col] == 1:  # Assuming 1 represents a free point
                self.samples.append((row, col))

    def gaussian_sample(self, n_pts):
        """Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph and samples
        self.graph.clear()
        self.samples = []

        mean_row = self.size_row // 2
        mean_col = self.size_col // 2
        std_dev = min(self.size_row, self.size_col) / 6  # Example std deviation

        count = 0
        while count < n_pts:
            row = int(np.random.normal(mean_row, std_dev))
            col = int(np.random.normal(mean_col, std_dev))
            if 0 <= row < self.size_row and 0 <= col < self.size_col and self.map_array[row, col] == 1:
                self.samples.append((row, col))
                count += 1

    def bridge_sample(self, n_pts):
        """Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()
        self.samples = []

        count = 0
        while count < n_pts:
            p1_row = np.random.randint(0, self.size_row)
            p1_col = np.random.randint(0, self.size_col)
            p2_row = np.random.randint(0, self.size_row)
            p2_col = np.random.randint(0, self.size_col)

            if self.check_collision((p1_row, p1_col), (p2_row, p2_col)):
                mid_row = (p1_row + p2_row) // 2
                mid_col = (p1_col + p2_col) // 2
                if self.map_array[mid_row, mid_col] == 1:
                    self.samples.append((mid_row, mid_col))
                    count += 1

    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos["start"] = (self.samples[-2][1], self.samples[-2][0])
        pos["goal"] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(
            self.graph, pos, node_size=3, node_color="y", edge_color="y", ax=ax
        )

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(
                self.graph,
                pos=pos,
                nodelist=self.path,
                node_size=8,
                node_color="b",
            )
            nx.draw_networkx_edges(
                self.graph,
                pos=pos,
                edgelist=final_path_edge,
                width=2,
                edge_color="b",
            )

        # draw start and goal
        nx.draw_networkx_nodes(
            self.graph,
            pos=pos,
            nodelist=["start"],
            node_size=12,
            node_color="g",
        )
        nx.draw_networkx_nodes(
            self.graph, pos=pos, nodelist=["goal"], node_size=12, node_color="r"
        )

        # show image
        plt.axis("on")
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):
        """Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        """
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01),
        #          (p_id0, p_id2, weight_02),
        #          (p_id1, p_id2, weight_12) ...]
        tree  =KDTree(self.samples)
        k_neighbors = 5
        pairs = []

        for p_id, point in enumerate(self.samples):
            # Query the k-nearest neighbors
            distances, indices = tree.query(point, k=k_neighbors + 1)  # +1 because point itself is included

            for dist, idx in zip(distances[1:], indices[1:]):  # skip the 0-th element because it's the point itself
                if not self.check_collision(point, self.samples[idx]):
                    # If there's no collision, store the pair and their distance
                    pairs.append((p_id, idx, dist))

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01),
        #                                     (p_id0, p_id2, weight_02),
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print(
            "The constructed graph has %d nodes and %d edges"
            % (n_nodes, n_edges)
        )

    def search(self, start, goal):
        """Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal nodes, edges of them
        and their nearest neighbors to graph for
        self.graph to search for a path.
        """
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(["start", "goal"])

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0),
        #                (start_id, p_id1, weight_s1),
        #                (start_id, p_id2, weight_s2) ...]

        start_pairs = []
        goal_pairs = []
        
        # Build K-D tree for efficient nearest neighbor search
        tree = KDTree(self.samples[:-2])  # Exclude start and goal from the tree
        k_neighbors = 5  # Let's consider 5 nearest neighbors for example
         # Find k-nearest neighbors for start and their distances
        distances_start, indices_start = tree.query(start, k=k_neighbors)
        for dist, idx in zip(distances_start, indices_start):
            if not self.check_collision(start, self.samples[idx]):
                start_pairs.append(("start", idx, dist))

        # Find k-nearest neighbors for goal and their distances
        distances_goal, indices_goal = tree.query(goal, k=k_neighbors)
        for dist, idx in zip(distances_goal, indices_goal):
            if not self.check_collision(goal, self.samples[idx]):
                goal_pairs.append(("goal", idx, dist))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(
                self.graph, "start", "goal"
            )
            path_length = (
                nx.algorithms.shortest_paths.weighted.dijkstra_path_length(
                    self.graph, "start", "goal"
                )
            )
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(["start", "goal"])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
