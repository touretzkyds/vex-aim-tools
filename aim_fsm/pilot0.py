"""
To avoid circular dependencies between pilot.fsm, doorpass.fsm, and
path_planner.py, we put some pilot classes here so everyone can import
them.

"""

from .utils import *
from .base import *
from .rrt import *
from .events import PilotEvent


#---------------- Pilot Exceptions ----------------

class PilotException(Exception):
    def __str__(self):
        return self.__repr__()

class InvalidPose(PilotException): pass
class CollisionDetected(PilotException): pass
class NotLocalized(PilotException): pass

# Note: StartCollides, GoalCollides, and MaxIterations exceptions are defined in rrt.py.



#---------------- Pilot Utility Nodes ----------------

class PilotCheckStart(StateNode):
    "Fails if rrt planner indicates start_collides"

    def start(self, event=None):
        super().start(event)
        pose = self.robot.pose
        start_node = RRTNode(x=pose.x, y=pose.y, q=pose.theta)
        try:
            self.robot.rrt.plan_path(start_node, None)
        except StartCollides as e:
            print('PilotCheckStart: Start collides!',e)
            self.post_event(PilotEvent(StartCollides, args=e.args))
            self.post_failure()
            return
        except Exception as e:
            print('PilotCheckStart: Unexpected planner exception',e)
            self.post_failure()
            return
        self.post_success()


class PilotCheckStartDetail(StateNode):
    "Posts collision object if rrt planner indicates start_collides"

    def start(self, event=None):
        super().start(event)
        pose = self.robot.pose
        start_node = RRTNode(x=pose.x, y=pose.y, q=pose.theta)
        try:
            self.robot.rrt.plan_path(start_node,start_node)
        except StartCollides as e:
            print('PilotCheckStartDetail: Start collides!',e)
            self.post_event(PilotEvent(StartCollides, args=e.args))
            self.post_data(e.args)
            return
        except Exception as e:
            print('PilotCheckStartDetail: Unexpected planner exception',e)
            self.post_failure()
            return
        self.post_success()

#---------------- Navigation Plan ----------------

class NavStep():
    DRIVE = "drive"
    DOORPASS = "doorpass"
    BACKUP = "backup"
    TURN_TO = "turn_to"

    def __init__(self, type, param):
        """For DRIVE and BACKUP types, param is a list of RRTNode instances.  The
        reason we group these into a list instead of having one node per step is that
        the DriveContinuous function is going to be interpolating over the entire sequence.
        For a DOORPASS step the param is the door object."""
        self.type = type
        self.param = param

    def __repr__(self):
        if self.type == NavStep.DOORPASS:
            pstring = self.param.id
        elif self.type == NavStep.DRIVE:
            psteps = [f'({node.x:.1f}, {node.y:.1f})' for node in self.param]
            pstring = '[' + ' '.join(psteps) + ']'
        elif self.type == NavStep.TURN_TO:
            pstring = f'{self.param * 180/pi:.1f} deg.'
        else:   # NavStep.BACKUP and anything else
            pstring = repr(self.param)
            if len(pstring) > 40:
                pstring = pstring[0:20] + ' ...' + pstring[-20:]
        return '<NavStep %s %s>' % (self.type, pstring)


class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    def __repr__(self):
        steps = [(('doorpass(%s)' % s.param.id) if s.type == NavStep.DOORPASS else s.type) for s in self.steps]
        return '<NavPlan %s>' % repr(steps)

    def extract_path(self):
        nodes = []
        for step in self.steps:
            if step.type in (NavStep.DRIVE, NavStep.BACKUP):
                nodes += step.param
        return nodes
