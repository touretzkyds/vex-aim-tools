"""
Create a dummy robot so we can use vex-aim-tools classes without
having to connect to a real robot.
"""

import asyncio

from .aim_kin import AIMKinematics
from .evbase import EventRouter
from .particle import SLAMParticleFilter
from .rrt import RRT
from .path_planner import PathPlanner
from .utils import PoseEstimate
from .worldmap import WorldMap

class SimRobot0():
    class Inertial():
        def get_heading(self): return 0

    def __init__(self):
        self.inertial = self.Inertial()
    def get_x_position(self): return 0
    def get_y_position(self): return 0

class SimRobot():
    def __init__(self, run_in_cloud=False):
        robot = self
        robot.use_shared_map = False
        robot.particle_viewer = None
        robot.path_viewer = None
        robot.path_planner = PathPlanner(robot)
        robot.robot0 = SimRobot0()

        robot.pose = PoseEstimate(0, 0, 0, 0)
        robot.holding = None

        if not run_in_cloud:
            robot.loop = asyncio.get_event_loop()
            robot.erouter = EventRouter(robot)

        robot.world_map = WorldMap(robot)
        robot.world_map.wall_marker_dict = dict()
        robot.particle_filter = SLAMParticleFilter(robot)
        robot.kine = AIMKinematics(robot)
        robot.rrt = RRT(robot)

    def set_pose(self, x, y, z, theta, reset_particles=True):
        self.pose = PoseEstimate(x, y, z, theta)
        if self.particle_filter and reset_particles:
            self.particle_filter.set_pose(x,y,theta)
