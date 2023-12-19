import matplotlib.pyplot as plt
import matplotlib.animation as manimation


class Planner:
    """A planner class that plans a path from start to goal
    in a certain map for a certain robot using a certain method.
    """

    def __init__(self, method, map_2d, robot):
        """Initialize with the method, robot, map, and obstacles."""
        self.method = method
        self.map = map_2d
        self.robot = robot

        # initialize the method with the map and robot
        self.method.set_map(self.map)
        self.method.set_robot(self.robot)

        # result container
        self.start = None
        self.goal = None
        self.solution = None

    def plan(self, start, goal):
        """Plan a path from start to goal."""
        self.start = start
        self.goal = goal
        self.solution = self.method.plan(self.start, self.goal)
        return self.solution

    def visualize(self, make_video=False):
        """Visualize the envrionment and solution."""
        # Draw the map
        ax, fig = self.map.visualize_map(return_ax=True)

        # Define the meta data for the movie
        FFMpegWriter = manimation.writers['ffmpeg']
        metadata = dict(title='Movie Test', artist='Matplotlib',
                        comment='An animation of the robot planning')
        writer = FFMpegWriter(fps=15, metadata=metadata)
        self.method.visualize_sampling_result(ax)

        # Draw the robot with solution configurations
        if self.solution != []:
               if make_video:
                   writer.setup(fig, "path.mp4", 100)

               # draw path
               for i in range(len(self.solution)):
                  if i == len(self.solution) - 1:
                       continue
                  p1 = self.robot.forward_kinematics(self.solution[i])[-1]
                  p2 = self.robot.forward_kinematics(self.solution[i + 1])[-1]
                  ax.plot(
                      [p1[0], p2[0]],
                      [p1[1], p2[1]],
                      color="gray",
                      linewidth=1,
                  )
                  if make_video:
                      writer.grab_frame()

               for i in range(len(self.solution)-1):
                   # draw robot
                   for c in self.robot.interpolate(self.solution[i], self.solution[i+1], num=10): 
                       self.robot.draw_robot(ax, c , edgecolor="blue")
                       if make_video:
                           writer.grab_frame()

        # Draw the start and goal configuration if provided
        if self.start is not None and self.goal is not None:
            self.robot.draw_robot(ax, self.start, edgecolor="red")
            self.robot.draw_robot(ax, self.goal, edgecolor="green")

        plt.axis("on")
        ax.tick_params(
            left=True, bottom=True, labelleft=True, labelbottom=True
        )
        plt.show()
