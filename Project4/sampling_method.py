from utils import endpoints_to_edges


class SamplingMethod():
    """A parent class for sampling methods (RRT, PRM, etc.)."""

    def __init__(self):
        self.map = None
        self.robot = None

    def set_map(self, map_2d):
        """Set the map for sampling."""
        self.map = map_2d
        if self.robot is not None:
            self.update_robot_limits()

    def set_robot(self, robot):
        """Set the robot for sampling."""
        self.robot = robot
        if self.map is not None:
            self.update_robot_limits()

    def update_robot_limits(self):
        """Update the robot limits based on the map boundary if needed."""
        for i in range(len(self.robot.limits)):
            if self.robot.limits[i][2] == "x":
                self.robot.limits[i] = (0, self.map.shape[0], "x")
            elif self.robot.limits[i][2] == "y":
                self.robot.limits[i] = (0, self.map.shape[1], "y")

    def check_collision(self, config1, config2):
        """Check if the path between two configurations collide with obstacles
        Use the robot's check_collision function
        """
        return self.robot.check_collision(
            config1, 
            config2, 
            self.map.corners, 
            self.map.obstacles, 
            self.map.obstacle_edges
        )

    def check_collision_config(self, config):
        """Check if a single configuration is colliding with obstacles
        Use the robot's check_collision_config function
        """
        return self.robot.check_collision_config(
            config, 
            self.map.corners, 
            self.map.obstacles, 
            self.map.obstacle_edges
        )

    def plan(self):
        """Sample a configuration."""
        raise NotImplementedError

    def visualize_sampling_result(self, ax):
        """Visualize the sampling result."""
        raise NotImplementedError
