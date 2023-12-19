import numpy as np

from map_2d import Map2D
from planner import Planner
from map_2d import Map2D
from robot import PointRobot, OmnidirectionalRobot, KinematicChain
from RRT import RRT
from PRM import PRM


if __name__ == "__main__":
    # Load the map
    # Map is loaded as a map size tuple map_2d.map_shape (x_lim, y_lim),
    # and a list of obstacles map_2d.obstacles [(x1, y1), (x2, y2), ...].
    map_2d = Map2D("maps/map.csv")

    # Define the robot with start and goal configurations
    robot = PointRobot()
    start = (20, 20)
    goal = (80, 80)
    #robot = OmnidirectionalRobot(width=2.5, height=5)
    #start = (20, 20, 0)
    #goal = (80, 80, np.pi / 2)
    #robot = KinematicChain(link_lengths=[15, 15, 15, 15], base=[50, 15])
    #start = (3, -0.1, -0.5, -0.5)
    #goal = (2, -0.4, -0.3, -0.1)

    # select the planner method
    # for the pointrobot and omnidirectionalrobot
    method = PRM(sampling_method="uniform", n_configs=300, kdtree_d=10)
    # method = PRM(sampling_method="gaussian", n_configs=300, kdtree_d=10)
    # method = PRM(sampling_method="bridge", n_configs=300, kdtree_d=10)
    # method =  RRT(sampling_method="RRT", n_configs=300, kdtree_d=10)
    # method = RRT(sampling_method="RRT_star", n_configs=300, kdtree_d=10)
    # method = RRT(sampling_method="Informed_RRT_star", n_configs=300, kdtree_d=10)

    # For the KinematicChain
    # method = PRM(sampling_method="uniform", n_configs=100, kdtree_d=np.pi)
    # method = PRM(sampling_method="gaussian", n_configs=200, kdtree_d=np.pi)
    # method = PRM(sampling_method="bridge", n_configs=200, kdtree_d=np.pi)
    # method = RRT(sampling_method="RRT", n_configs=300, kdtree_d=np.pi)
    # method = RRT(sampling_method="RRT_star", n_configs=300, kdtree_d=np.pi)
    # method = RRT(sampling_method="Informed_RRT_star", n_configs=300, kdtree_d=np.pi)

    # Initialize the planner
    planner = Planner(method, map_2d, robot)

    # planning
    planner.plan(start, goal)
    #If make_video true a video it will create file path.mp4. 
    #NOTE: Creating the video takes a considerable amount of time.
    planner.visualize(make_video=False)
