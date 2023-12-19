import numpy as np
import networkx as nx
from scipy import spatial

from utils import interpolate_angle
from sampling_method import SamplingMethod


class PRM(SamplingMethod):
    """Probabilistic Roadmap (PRM) planner"""

    def __init__(self, sampling_method="uniform", n_configs=1000, kdtree_d=10):
        """Specifying number of configs and sampling method to initialize
        arguments:
            sampling_method - name of the chosen sampling method
            n_configs - number of configs to sample
            kdtree_d - the distance for kdtree to search for nearest neighbors
        """
        super().__init__()
        self.sampling_method = sampling_method
        self.n_configs = n_configs
        self.kdtree_d = kdtree_d

        # kd tree spatial.KDTree([[]]), euclidean distance only
        self.kdtree = None
        self.samples = []  # list of sampled configs
        self.graph = nx.Graph()  # constructed graph
        self.solution = []  # list of nodes of the found solution

    def get_sample(self):
        """Get a sample configuration"""
        # Generate a random config
        sample = []
        # Sample in each dimension
        for i in range(self.robot.dof):
            sample.append(
                np.random.uniform(
                    self.robot.limits[i][0], 
                    self.robot.limits[i][1]
                )
            )
        return np.array(sample)

    def get_gaussian_offset_sample(self, sample, scale=10):
        """Get another sample at some distance from the given sample
        arguments:
            sample - the given sample
            scale - the standard deviation of the gaussian distribution
                    is defined as limit / scale

        return:
            a sample at some distance from the given sample
        """
        diff = []
        # Gaussian sample in each dimension 
        # with mean 0 and std limit / scale
        for i in range(self.robot.dof):
            diff.append(
                np.random.normal(
                    0.0, 
                    np.abs(
                        self.robot.limits[i][1] - self.robot.limits[i][0]
                    ) / scale
                )
            )
        return sample + np.array(diff)

    def uniform_sample(self, n_configs):
        """Use uniform sampling and store valid configs
        arguments:
            n_configs - number of configs to sample

        check collision and store valide configs in self.samples
        """
        while len(self.samples) < n_configs:
            # Generate a uniform random config
            sample = self.get_sample()
            # Check obstacle
            if not self.check_collision_config(sample):
                self.samples.append(sample)

    def gaussian_sample(self, n_configs, prob=0.05):
        """Use gaussian sampling and store valid configs
        arguments:
            n_configs - number of configs to sample
            prob - probability of sampling a config with uniform sampling,
                   to avoid long sampling time in certain maps

        check collision and store valide configs in self.samples
        """
        while len(self.samples) < n_configs:
            # Generate a uniform random config
            sample = self.get_sample()

            # Use uniform sampling with probability prob
            if np.random.uniform() < prob:
                # Check obstacle
                if not self.check_collision_config(sample):
                    self.samples.append(sample)
                continue

            # Use gaussian sampling
            # Generate a new random config at some distance
            sample2 = self.get_gaussian_offset_sample(sample)

            # Check if the config is close to an obstacle, i.e.,
            # one of them is obstacle and the other is free space
            collision1 = self.check_collision_config(sample)
            collision2 = self.check_collision_config(sample2)
            if (collision1 and not collision2):
                self.samples.append(sample2)
            elif (collision2 and not collision1):
                self.samples.append(sample)

    def bridge_sample(self, n_configs, prob=0.05):
        """Use bridge sampling and store valid configs
        arguments:
            n_configs - number of configs to sample
            prob - probability of sampling a config with uniform sampling,
                   to avoid long sampling time in certain maps

        check collision and store valide configs in self.samples
        """
        while len(self.samples) < n_configs:
            # Generate a uniform random config
            sample = self.get_sample()

            # Use uniform sampling with probability prob
            if np.random.uniform() < prob:
                # Check obstacle
                if not self.check_collision_config(sample):
                    self.samples.append(sample)
                continue

            # Use gaussian sampling
            # Generate a new random config at some distance
            sample2 = self.get_gaussian_offset_sample(sample)

            # Check if it is the "bridge" form, i.e.,
            # the center is not obstacle but both ends are obstacles
            # Both are obstacles
            collision1 = self.check_collision_config(sample)
            collision2 = self.check_collision_config(sample2)
            if (collision1 and collision2):
                # The mid-sample is not an obstacle
                mid_sample = sample
                for i in range(self.robot.dof):
                    # position
                    if (self.robot.limits[i][2] != "r"):
                        mid_sample[i] = np.mean([sample[i], sample2[i]])
                    # rotation
                    else:
                        mid_sample[i] = interpolate_angle(
                            sample[i], sample2[i], 3
                        )[1]
                if (not self.check_collision_config(mid_sample)):
                    self.samples.append(mid_sample)

    def add_vertices_pairs(self, pairs):
        """Add pairs of vertices to graph as weighted edge
        arguments:
            pairs - pairs of vertices of the graph

        check collision, compute weight and add valide edges to self.graph
        """
        for pair in pairs:
            if pair[0] == "start":
                config1 = self.samples[-2]
            elif pair[0] == "goal":
                config1 = self.samples[-1]
            else:
                config1 = self.samples[pair[0]]
            config2 = self.samples[pair[1]]

            if not self.check_collision(config1, config2):
                d = self.robot.distance(config1, config2)
                self.graph.add_weighted_edges_from([(pair[0], pair[1], d)])

    def connect_vertices(self):
        """Add nodes and edges to the graph from sampled configs
        arguments:
            kdtree_d - the distance for kdtree to search for nearest neighbors

        Add nodes to graph from self.samples
        Build kdtree to find neighbor pairs and add them to graph as edges
        """
        # Finds k nearest neighbors
        # kdtree
        self.kdtree = spatial.cKDTree(self.samples)
        pairs = self.kdtree.query_pairs(self.kdtree_d)

        # Add the neighbor to graph
        self.graph.add_nodes_from(range(len(self.samples)))
        self.add_vertices_pairs(pairs)

    def sample(self):
        """Construct a graph for PRM
        arguments:
            n_configs - number of configs try to sample,
                    not the number of final sampled configs
            sampling_method - name of the chosen sampling method
            kdtree_d - the distance for kdtree to search for nearest neighbors

        Sample configs, connect, and add nodes and edges to self.graph
        """
        # Initialization
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample using given methods
        # The results will be stored in self.samples
        if self.sampling_method == "uniform":
            self.uniform_sample(self.n_configs)
        elif self.sampling_method == "gaussian":
            self.gaussian_sample(self.n_configs)
        elif self.sampling_method == "bridge":
            self.bridge_sample(self.n_configs)

        # Connect the samples in self.samples
        # The results will be stored in self.graph
        self.connect_vertices()

    def plan(self, start, goal, redo_sampling=True):
        """Search for a path in graph given start and goal location
        Temporary add start and goal node,
        edges of them and their nearest neighbors to graph.
        Use graph search algorithm to search for a path.

        arguments:
            start - start configuration
            goal - goal configuration
            redo_sampling - whether to rerun the sampling phase
        """
        self.solution = []
        # Sampling and building the roadmap
        if redo_sampling:
            self.sample()

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        self.graph.add_nodes_from(["start", "goal"])

        # Connect start and goal nodes to the surrounding nodes
        start_goal_tree = spatial.cKDTree([start, goal])
        neighbors = start_goal_tree.query_ball_tree(self.kdtree, 2*self.kdtree_d)
        start_pairs = [["start", neighbor] for neighbor in neighbors[0]]
        goal_pairs = [["goal", neighbor] for neighbor in neighbors[1]]

        # Add the edge to graph
        self.add_vertices_pairs(start_pairs)
        self.add_vertices_pairs(goal_pairs)

        # Seach using Dijkstra
        found = False
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(
                self.graph, "start", "goal"
            )
            length = (
                nx.algorithms.shortest_paths.weighted.dijkstra_path_length(
                    self.graph, "start", "goal"
                )
            )
            found = True
            num_nodes = self.graph.number_of_nodes()
            print("The constructed graph has %d of nodes" % num_nodes)
            print("The path length is %.2f" % length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(["start", "goal"])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)

        # Return the final solution
        if found:
            self.solution = [self.samples[i] for i in self.path[1:-1]]
            self.solution.insert(0, start)
            self.solution.append(goal)
        return self.solution

    def visualize_sampling_result(self, ax):
        """ Visualization the sampling result."""
        # Get node 2d position by computing robot forward kinematics,
        # assuming the last endpoint of the robot is the position.
        node_names = list(range(len(self.samples)))
        end_points = [
            self.robot.forward_kinematics(config)[-1] 
            for config in self.samples
        ]
        positions = dict(zip(node_names, end_points))

        # Draw constructed graph
        nx.draw(
            self.graph,
            positions,
            node_size=5,
            node_color="g",
            edge_color="y",
            alpha=0.3,
            ax=ax
        )
