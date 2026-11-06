"""
Path planner using RRT and Wavefront algorithms.
"""

from math import pi, sin, cos, sqrt, nan, isnan
from multiprocessing import Process

#from .nodes import LaunchProcess
from .base import StateNode
from .utils import Pose
from .events import DataEvent, PilotEvent
from .pilot0 import NavPlan, NavStep, NotLocalized
from .worldmap import WorldObject, BarrelObj, SportsBallObj, AprilTagObj, DoorwayObj, ArucoMarkerObj, RoomObj
from .rrt import RRT, RRTNode, StartCollides, GoalCollides, GoalUnreachable
from .wavefront import WaveFront
from .geometry import wrap_angle, segment_intersect_test
from .particle import ParticleFilter

from . import rrt

class PathPlanner():
    """This path planner can be called directly, or it can be used inside
    a PathPlannerProcess node that runs the heavy lifting portion of
    the algorithm in a child process.  Because child processes in
    Windows don't share memory with the parent, we must transmit
    certain data to the child as parameters during process creation.
    But only structures that are pickle-able can be sent.  The
    setup_problem() method sets up those structures, and do_planning()
    uses them to do the work.  If we don't want to run in a separate
    process then we can use plan_path_this_process() to call both
    methods in the main process and return the result.
    """

    # Note: the obstacle inflation parameter is a radius, not a diameter.

    # We need fat obstacles for the wavefefront algorithm because the
    # robot itself is treated as a point.
    fat_obstacle_inflation = 40  # must be << pilot's escape_distance
    fat_wall_inflation = 35
    fat_doorway_adjustment = -62

    # We use skinny obstacles for the RRT because we model the robot's
    # shape explicitly, so it's not just a point.
    skinny_obstacle_inflation = 15
    skinny_wall_inflation = 10
    skinny_doorway_adjustment = 0

    def __init__(self, robot):
        self.robot = robot
        self.wf = None

    @staticmethod
    def plan_path_this_process(robot, goal_object, use_doorways=True):
        # Get pickle-able data structures
        (start_node, goal_shape, robot_parts, bbox,
         fat_obstacles, skinny_obstacles, doorway_list, need_grid_display) = \
            __class__.setup_problem(goal_object, robot, use_doorways)
        # Do the actual path planning
        result = \
            __class__.do_planning(robot.rrt, start_node, goal_shape,
                                  fat_obstacles, skinny_obstacles, doorway_list,
                                  need_grid_display)
        if isinstance(result, PilotEvent):
            grid_display = result.args['grid_display']
        elif isinstance(result, DataEvent):
            (navplan, grid_display) = result.data
        else:
            raise ValueError('Bad result type:', result)
        robot.rrt.grid_display = grid_display
        return result

    @staticmethod
    def setup_problem(goal_object, robot, use_doorways):
        """Calculate values from world map in main process since the map won't
        be available in the child process."""

        # Fat obstacles and narrow doorways for WaveFront
        robot.rrt.generate_obstacles(goal_object,
                                     PathPlanner.fat_obstacle_inflation,
                                     PathPlanner.fat_wall_inflation,
                                     PathPlanner.fat_doorway_adjustment)
        fat_obstacles = robot.rrt.obstacles

        # Skinny obstacles and normal doorways for RRT
        robot.rrt.generate_obstacles(goal_object,
                                     PathPlanner.skinny_obstacle_inflation,
                                     PathPlanner.skinny_wall_inflation,
                                     PathPlanner.skinny_doorway_adjustment)
        skinny_obstacles = robot.rrt.obstacles

        start_node = RRTNode(x=robot.pose.x, y=robot.pose.y, q=robot.pose.theta)

        if isinstance(goal_object, BarrelObj):
            goal_shape = RRT.generate_barrel_obstacle(goal_object, 0)
        elif isinstance(goal_object, SportsBallObj):
            goal_shape = RRT.generate_ball_obstacle(goal_object, 0)
        elif isinstance(goal_object, AprilTagObj):
            goal_shape = RRT.generate_apriltag_obstacle(goal_object, 0)
        elif isinstance(goal_object, DoorwayObj):
            goal_shape = RRT.generate_doorway_obstacle(goal_object,0)
        elif isinstance(goal_object, ArucoMarkerObj):
            goal_shape = RRT.generate_aruco_obstacle(goal_object, 0)
        elif isinstance(goal_object, RoomObj):
            goal_shape = RRT.generate_room_obstacle(goal_object)
        else:
            raise ValueError("Can't convert path planner goal %s to shape." % goal_object)

        robot.rrt.goal_obstacle = goal_shape
        bbox = robot.rrt.compute_bounding_box()

        robot_parts = robot.rrt.make_robot_parts(robot)

        if use_doorways:
            doorway_list = robot.world_map.generate_doorway_list()
        else:
            doorway_list = []  # don't truncate path at doorways in simulator

        need_grid_display = robot.path_viewer is not None

        return (start_node, goal_shape, robot_parts, bbox,
                fat_obstacles, skinny_obstacles, doorway_list, need_grid_display)

    @staticmethod
    def do_planning(rrt_instance, start_node, goal_shape,
                    fat_obstacles, skinny_obstacles, doorway_list, need_grid_display):
        """Does the heavy lifting; may be called in a child process."""

        escape_options = (
                           # angle       distance(mm)
                           (0,            40),
                           (+30/180*pi,   50),
                           (-30/180*pi,   50),
                           (pi,           40),
                           (pi,           80),  # if we're wedged between two objects
                           (+60/180*pi,   80),
                           (-60/180*pi,   80),
                           (+pi/2,        70),
                           (-pi/2,        70)
        )

        rrt_instance.obstacles = skinny_obstacles
        start_escape_move = None

        wf = WaveFront(rrt_instance.robot, bbox=rrt_instance.bbox)
        rrt_instance.robot.path_planner.wf = wf
        for obstacle in fat_obstacles:
            wf.add_obstacle(obstacle)

        collider = rrt_instance.collides(start_node)
        if not collider:
            collider = wf.check_start_collides(start_node.x, start_node.y)

        distance_to_goal = sqrt((start_node.x - goal_shape.center[0,0])**2 + (start_node.y - goal_shape.center[1,0])**2)
        distance_threshold = (rrt_instance.robot.kine.body_diameter/2) + 15 # fudge factor
        too_close_to_goal = distance_to_goal <= distance_threshold
        if collider or too_close_to_goal:
          if too_close_to_goal or collider.obstacle_id is goal_shape.obstacle_id:  # We're already at the goal
            print("path_planner: We're already at the goal.")
            step = NavStep(NavStep.DRIVE, [RRTNode(x=start_node.x, y=start_node.y)])
            navplan = NavPlan([step])
            grid_display = None if not need_grid_display else wf.grid
            result = (navplan, grid_display)
            return DataEvent(result)
          else:
            # Find an escape move from this collider
            q = start_node.q
            for (phi, escape_distance) in escape_options:
                if phi != pi:
                    new_q = wrap_angle(q + phi)
                    escape_type = NavStep.DRIVE
                else:
                    new_q = q   # drive backwards on current heading
                    escape_type = NavStep.BACKUP
                new_start = RRTNode(x=start_node.x + escape_distance*cos(q+phi),
                                    y=start_node.y + escape_distance*sin(q+phi),
                                    q=new_q)
                collider2 = rrt_instance.collides(new_start)
                #print('trying escape', new_start, 'collision:', collider2)
                if not collider2  and \
                   not wf.check_start_collides(new_start.x,new_start.y):
                    start_escape_move = (escape_type, phi, start_node, new_start)
                    start_node = new_start
                    print('Path planner found escape move from', collider, 'using:', start_escape_move)
                    break
            if start_escape_move is None:
                print('PathPlanner: Start collides!', collider)
                return PilotEvent(StartCollides,collider=collider,grid_display=None,text="start collides")

        # Run the wavefront path planner
        rrt_instance.obstacles = fat_obstacles
        if goal_shape.obstacle_id.startswith('Room'):
            offsets = [1, -25, -1]   # try different spots in the room
        else:
            offsets = [None]
        for i in range(len(offsets)):
            offset = offsets[i]
            if i > 0:
                wf = WaveFront(rrt_instance.robot, bbox=rrt_instance.bbox)  # need a fresh grid
            wf.set_goal_shape(goal_shape, offset, obstacle_inflation=0)
            # obstacles come after the goal so they can overwrite goal pixels
            for obstacle in fat_obstacles:
                wf.add_obstacle(obstacle)
            wf_start = (start_node.x, start_node.y)
            goal_found = wf.propagate(*wf_start)
            if goal_found: break
            print('Wavefront planning failed with offset', offset)
        grid_display = wf.grid if need_grid_display else None
        if goal_found is None:
            print('PathPlanner wavefront: goal unreachable!')
            return PilotEvent(GoalUnreachable, grid_display=grid_display, text='unreachable')

        # Extract and smooth the path
        coords_pairs = wf.extract(goal_found, wf_start)
        rrt_instance.path = rrt_instance.coords_to_path(coords_pairs)
        rrt_instance.obstacles = skinny_obstacles
        #rrt_instance.obstacles = fat_obstacles
        rrt_instance.smooth_path()

        # If the path ends in a collision according to the RRT, back off
        while True: # len(rrt_instance.path) > 2:
          last_node = rrt_instance.path[-1]
          collider = rrt_instance.collides(last_node)
          if collider:
              print('Collision detected after path smoothing:', collider)
              break
              # rrt_instance.path = rrt_instance.path[:-1]
          else:
            break

        # Construct the navigation plan
        navplan = PathPlanner.from_path(rrt_instance.path, doorway_list)

        # Insert the StartCollides escape move if there is one
        if start_escape_move:
            escape_type, phi, start, new_start = start_escape_move
            if escape_type == NavStep.BACKUP:
                escape_step = NavStep(NavStep.BACKUP, (RRTNode(x=new_start.x, y=new_start.y),))
                navplan.steps.insert(0, escape_step)
            elif navplan.steps[0].type == NavStep.DRIVE:
                navplan.steps[0].param.insert(0, RRTNode(x=start.x, y=start.y))
            else:
                # Shouldn't get here, but just in case
                print("Shouldn't end up here!", navplan.steps[0])
                escape_step = NavStep(NavStep.DRIVE,
                                      (RRTNode(x=start.x, y=start.y),
                                       RRTNode(x=new_start.x, y=new_start.y)))
                navplan.steps.insert(0, escape_step)

        # Return the navigation plan
        print('navplan=',navplan, '   steps=',navplan.steps)
        result = (navplan, grid_display)
        return DataEvent(result)

    @staticmethod
    def intersects_doorway(node1, node2, doorways):
        for door in doorways:
            p1 = (node1.x, node1.y)
            p2 = (node2.x, node2.y)
            p3 = door[1][0]
            p4 = door[1][1]
            result = segment_intersect_test(p1, p2, p3, p4)
            #label = '**INTERSECTS**' if result else 'no_int:'
            #print(label,door[0].id,' ( %.1f, %.1f )<=>( %.1f, %.1f )  vs  ( %.1f, %.1f )<=>( %.1f, %.1f )' % (p1+p2+p3+p4))
            if result:
                return door[0]
        return None

    @staticmethod
    def from_path(path, doorways):
        # Consider each path segment (defined by start and end
        # RRTNodes) and see if it crosses a doorway.
        door = None
        i = 0  # in case len(path) is 1 and we skip the for loop
        pt1 = path[i]
        for i in range(1, len(path)):
            pt2 = path[i]
            door = PathPlanner.intersects_doorway(pt1,pt2,doorways)
            if door:
                i -= 1
                break
            else:
                pt1 = pt2

        # If no doorway, we're good to go.  See if we need to turn at the end.
        if door is None:
            if len(path) > 1 and path[-1].x == path[-2].x and path[-1].y == path[-2].y and not isnan(path[-1].q):
                all_but_last = path[:-1]
                drive_step = NavStep(NavStep.DRIVE, all_but_last)
                turn_step = NavStep(NavStep.TURN_TO, path[-1].q)
                plan = NavPlan([drive_step, turn_step])
            else:
                drive_step = NavStep(NavStep.DRIVE, path)
                plan = NavPlan([drive_step])
            return plan

        # Truncate the path at the doorway, and adjust to make sure
        # we're outside the approach gate.
        start_point = (pt1.x, pt1.y)
        DELTA = 15 # mm
        from .pilot import DoorPass
        gate, side = DoorPass.calculate_gate(start_point, door, DoorPass.OUTER_GATE_DISTANCE + DELTA)
        (dx,dy) = (door.pose.x, door.pose.y)
        (gx,gy) = (gate[0],gate[1])
        gate_node = RRTNode(x=gx, y=gy)
        print('door=', door, 'gate_node=', gate_node)

        while i > 0:
            (px,py) = (path[i].x, path[i].y)
            if ((px-dx)**2 + (py-dy)**2) >  (DoorPass.OUTER_GATE_DISTANCE + DELTA)**2:
                break
            i -= 1

        # For now, just truncate the path and insert an approach gate node.
        new_path = path[0:i+1]
        new_path.append(gate_node)
        step1 = NavStep(NavStep.DRIVE, new_path)
        step2 = NavStep(NavStep.DOORPASS, door)
        plan = NavPlan([step1, step2])
        return plan

class PathPlanToObject(StateNode):
    "goal_spec is either a WorldObject or a string (object id)"

    MIN_DISTANCE_THRESHOLD = 10 # mm

    def __init__(self, goal_spec=None):
        super().__init__()
        self.goal_spec = goal_spec

    def start(self,event=None):
        super().start(event)
        if isinstance(event, DataEvent):
            print(f'PathPlanToObject got {event} with {event.data}')
            if isinstance(event.data, (WorldObject, str)):
                self.goal_spec = event.data
            else:
                raise ValueError('DataEvent must be a WorldObject or string:', event)
        if isinstance(self.goal_spec, WorldObject):
            self.goal_obj = self.goal_spec
        elif self.goal_spec in self.robot.world_map.objects:
            self.goal_obj = self.robot.world_map.objects[self.goal_spec]
        else:
            raise ValueError('No world object with this id:', self.goal_spec)
        if self.robot.particle_filter.state != ParticleFilter.LOCALIZED:
            print('PathPlanToOjectNode: Robot not localized!')
            self.post_event(PilotEvent(NotLocalized))
            return
        distance = sqrt((self.goal_obj.pose.x - self.robot.pose.x)**2 + (self.goal_obj.pose.y - self.robot.pose.y)**2)
        if distance <= self.MIN_DISTANCE_THRESHOLD:
            print(f'PathPlanToObject: distance {distance:.1f} mm already close enough')
            self.post_completion()
            return
        else:
            print(f'PathPlanToObject: {distance = :.1f} mm')
        result = self.robot.path_planner.plan_path_this_process(self.robot, self.goal_obj)
        self.post_event(result)

#----------------------------------------------------------------

# This code is for running the path planner in a child process.

"""
class PathPlannerProcess(LaunchProcess):
    def start(self, event=None):
        if not isinstance(event,DataEvent):
            raise ValueError('PathPlanner node must be invoked with a DataEvent for the goal.')
        goal_object = event.data
        if not isinstance(goal_object, WorldObject):
            raise ValueError('Path planner goal %s is not a WorldObject' % goal_object)
        self.goal_object = goal_object
        self.print_trace_message('started:', 'goal=%s' % goal_object)
        super().start(event)  # will call create_process

    def create_process(self, reply_token):
        use_doorways = True   # assume we're running on the robot
        (start_node, goal_shape, robot_parts, bbox,
         fat_obstacles, skinny_obstacles, doorway_list, need_grid_display) = \
            PathPlanner.setup_problem(self.goal_object, self.robot, use_doorways)
        p = Process(target=self.__class__.process_workhorse,
                    args = [reply_token,
                            start_node, goal_shape, robot_parts, bbox,
                            fat_obstacles, skinny_obstacles, doorway_list,
                            need_grid_display])
        return p

    @staticmethod
    def process_workhorse(reply_token, start_node, goal_shape, robot_parts, bbox,
                          fat_obstacles, skinny_obstacles, doorway_list, need_grid_display):
        rrt_instance = RRT(robot_parts=robot_parts, bbox=bbox)
        result = \
            PathPlanner.do_planning(rrt_instance, start_node, goal_shape,
                                    fat_obstacles, skinny_obstacles, doorway_list,
                                    need_grid_display)
        __class__.post_event(reply_token, result)
"""
