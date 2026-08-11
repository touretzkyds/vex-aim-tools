import math
from math import nan, sqrt
import time
import sys
import asyncio
import copy

from .base import *
from .utils import Pose
from .rrt import *
from .nodes import *
from .events import PilotEvent
from .transitions import *
from .worldmap import WorldObject, DoorwayObj
from .path_planner import PathPlanner, PathPlanToObject #, PathPlannerProcess
from .particle import *
from .geometry import *
from .pilot0 import *
from .camera import AIVISION_RESOLUTION_SCALE

#---------------- Pilot Events ----------------

class ParentPilotEvent(StateNode):
    """Receive a PilotEvent and repost it from the receiver's parent. This allows
     derived classes that use the Pilot to make its PilotEvents visible."""
    def start(self,event):
        super().start(event)
        if not isinstance(event,PilotEvent):
            raise TypeError("ParentPilotEvent must be invoked with a PilotEvent, not %s" % event)
        if 'grid_display' in event.args:
            self.robot.rrt.grid_display = event.args['grid_display']
        event2 = PilotEvent(event.status)
        event2.args = event.args
        self.parent.post_event(event2)

#---------------- PilotBase ----------------

class PilotBase(StateNode):

    """Base class for PilotToObject, PilotToPose, etc."""

    class ClearDisplays(StateNode):
        def start(self,event=None):
            super().start()
            if self.robot.path_viewer:
                self.robot.path_viewer.clear()

    class SendObject(StateNode):
        def start(self,event=None):
          super().start()
          object = self.parent.object
          #if object.pose_confidence < 0:
          #   self.parent.post_event(PilotEvent(NotLocalized,object=object))
          #   self.parent.post_failure()
          #   return
          self.post_data(self.parent.object)

    class ReceivePlan(StateNode):
        def start(self, event=None):
            super().start(event)
            if not isinstance(event, DataEvent):
                raise ValueError(event)
            (navplan, grid_display) = event.data

            self.robot.rrt.draw_path = navplan.extract_path()
            #print('ReceivePlan: draw_path=', self.robot.world.rrt.draw_path)
            self.robot.rrt.grid_display = grid_display
            self.parent.navplan = navplan
            self.post_data(navplan)

    class PilotExecutePlan(StateNode):
        def start(self, event=None):
            if not self.parent.execute:
                self.parent.post_completion()
                return
            if isinstance(event, DataEvent) and isinstance(event.data, NavPlan):
                self.parent.navplan = event.data
            self.navplan = self.parent.navplan
            self.index = 0  # skip initial node which should match our pose
            super().start(event)

        class DispatchStep(StateNode):
            def start(self, event=None):
                super().start(event)
                step = self.parent.navplan.steps[self.parent.index]
                self.post_data(step.type)

        class ExecuteDrive(StateNode):
            class TurnStep(ActionNode):
                def start(self,event=None):
                    super().start(event)
                    pose = self.robot.pose
                    dest = self.parent.drive_steps[self.parent.index]
                    print(f'ExecuteDrive: dest={dest}  pose={pose}')
                    dx = dest.x - pose.x
                    dy = dest.y - pose.y
                    angle_rads = wrap_angle(atan2(dy,dx) - pose.theta)
                    print(f'Pilot TurnStep turning by {angle_rads*180/pi : .1f} deg.')
                    self.robot.actuators['drive'].turn(self, angle_rads)
                    
            class ForwardStep(ActionNode):
                def start(self,event=None):
                    super().start(event)
                    pose = self.robot.pose
                    dest = self.parent.drive_steps[self.parent.index]
                    dx = dest.x - pose.x
                    dy = dest.y - pose.y
                    distance_mm = (dx**2 + dy**2) ** 0.5
                    self.robot.actuators['drive'].forward(self,distance_mm)
                    
            class NextStep(StateNode):
                def start(self,event=None):
                    super().start(event)
                    self.parent.index += 1
                    if self.parent.index >= len(self.parent.drive_steps):
                        self.post_completion()
                    else:
                        self.post_success()

            def start(self, event=None):  # method of ExecutePlan
                step = self.parent.navplan.steps[self.parent.index]
                self.drive_steps = step.param
                self.index = 0  # skip the first entry, which should be current pose
                print('ExecutePlan steps=', self.drive_steps)
                super().start(event)

            def setup(self):
                #                 # PilotExecutePlan.ExecuteDrive machine
                #                 # We need separate turn and forward steps because turning
                #                 # while driving is not yet supported in aim.py.
                # 
                #                 next: self.NextStep()
                #                 next =C=> ParentCompletes()
                #                 next =S=> turn
                # 
                #                 turn: self.TurnStep() =C=> forward
                # 
                #                 forward: self.ForwardStep() =C=> next # Print('tm to continue drive...') =TM=> next
                # 
                
                # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
                
                next = self.NextStep() .set_name("next") .set_parent(self)
                parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
                turn = self.TurnStep() .set_name("turn") .set_parent(self)
                forward = self.ForwardStep() .set_name("forward") .set_parent(self)
                
                completiontrans1 = CompletionTrans() .set_name("completiontrans1")
                completiontrans1 .add_sources(next) .add_destinations(parentcompletes1)
                
                successtrans1 = SuccessTrans() .set_name("successtrans1")
                successtrans1 .add_sources(next) .add_destinations(turn)
                
                completiontrans2 = CompletionTrans() .set_name("completiontrans2")
                completiontrans2 .add_sources(turn) .add_destinations(forward)
                
                completiontrans3 = CompletionTrans() .set_name("completiontrans3")
                completiontrans3 .add_sources(forward) .add_destinations(next)
                
                return self

        class ExecuteTurnTo(Turn):
            def start(self, event):
                step = self.parent.navplan.steps[self.parent.index]
                target_heading = step.param
                self.angle_deg = wrap_angle(target_heading - self.robot.pose.theta) * 180/pi
                print(f'ExecuteTurnTo: target heading = {target_heading*180/pi:.1f} deg., ' +
                      f'current heading = {self.robot.pose.theta*180/pi:.1f} deg., ' +
                      f'turn angle = {self.angle_deg:.1f} .deg')
                super().start(event)


        class ExecuteDoorPass(StateNode):
            """ Greasy hack to get around circular dependency between Pilot and DoorPass """
            def start(self, event=None):
                original_doorpass = self.children['doorpass']
                print(f'{original_doorpass=}')
                if not isinstance(original_doorpass, DoorPass):
                    global pilot_global_doorpass_node  # set up in simple_cli
                    new_doorpass = copy.copy(pilot_global_doorpass_node)
                    print(f'{new_doorpass=}')
                    new_doorpass.parent = original_doorpass.parent
                    new_doorpass.transitions = original_doorpass.transitions
                    for t in new_doorpass.transitions:
                        t.sources = [new_doorpass]
                    new_doorpass.children = dict()
                    new_doorpass.setup()
                    new_doorpass.start_node = new_doorpass.children[pilot_global_doorpass_node.start_node.name]
                    self.children['doorpass'] = new_doorpass
                    self.start_node = new_doorpass
                    print(f'{new_doorpass=}')
                step = self.parent.navplan.steps[self.parent.index]
                self.children['doorpass'].door_spec = step.param
                super().start()

            def setup(self):
                #                 # PilotExecutePlan.ExecuteDoorPass machine
                #                 doorpass: StateNode()  # dummy to be replaced by start() due to circularity
                #                 doorpass =C=> ParentCompletes()
                #                 doorpass =F=> ParentFails()
                
                # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
                
                doorpass = StateNode() .set_name("doorpass") .set_parent(self)
                parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
                parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
                
                completiontrans4 = CompletionTrans() .set_name("completiontrans4")
                completiontrans4 .add_sources(doorpass) .add_destinations(parentcompletes2)
                
                failuretrans1 = FailureTrans() .set_name("failuretrans1")
                failuretrans1 .add_sources(doorpass) .add_destinations(parentfails1)
                
                return self

        class ExecuteBackup(Forward):
            def start(self, event=None):
                step = self.parent.navplan.steps[self.parent.index]
                if len(step.param) > 1:
                    print('***** WARNING: extra backup steps not being processed *****')
                node = step.param[0]
                dx = node.x - self.robot.pose.x
                dy = node.y - self.robot.pose.y
                self.distance_mm = - sqrt(dx*dx + dy*dy)
                super().start(event)

        class NextStep(StateNode):
            def start(self, event=None):
                super().start(event)
                self.parent.index += 1
                if self.parent.index < len(self.parent.navplan.steps):
                    self.post_success()
                else:
                    self.post_completion()

        def setup(self):
            #             # PilotExecutePlan machine
            # 
            #             dispatch: self.DispatchStep()
            #             dispatch =D(NavStep.DRIVE)=> drive
            #             dispatch =D(NavStep.TURN_TO)=> turn_to
            #             dispatch =D(NavStep.DOORPASS)=> doorpass
            #             dispatch =D(NavStep.BACKUP)=> backup
            # 
            #             drive: self.ExecuteDrive()
            #             drive =C=> next
            #             drive =F=> ParentFails()
            # 
            #             turn_to: self.ExecuteTurnTo()
            #             turn_to =C=> next
            #             turn_to =F=> ParentFails()
            # 
            #             doorpass: self.ExecuteDoorPass()
            #             doorpass =C=> next
            #             doorpass =F=> ParentFails()
            # 
            #             backup: self.ExecuteBackup()
            #             backup =C=> next
            #             backup =F=> ParentFails()
            # 
            #             #next: Print('tm to continue...') =TM=> next2
            # 
            #             next: self.NextStep()
            #             next =S=> dispatch
            #             next =C=> ParentCompletes()
            
            # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
            
            dispatch = self.DispatchStep() .set_name("dispatch") .set_parent(self)
            drive = self.ExecuteDrive() .set_name("drive") .set_parent(self)
            parentfails2 = ParentFails() .set_name("parentfails2") .set_parent(self)
            turn_to = self.ExecuteTurnTo() .set_name("turn_to") .set_parent(self)
            parentfails3 = ParentFails() .set_name("parentfails3") .set_parent(self)
            doorpass = self.ExecuteDoorPass() .set_name("doorpass") .set_parent(self)
            parentfails4 = ParentFails() .set_name("parentfails4") .set_parent(self)
            backup = self.ExecuteBackup() .set_name("backup") .set_parent(self)
            parentfails5 = ParentFails() .set_name("parentfails5") .set_parent(self)
            next = self.NextStep() .set_name("next") .set_parent(self)
            parentcompletes3 = ParentCompletes() .set_name("parentcompletes3") .set_parent(self)
            
            datatrans1 = DataTrans(NavStep.DRIVE) .set_name("datatrans1")
            datatrans1 .add_sources(dispatch) .add_destinations(drive)
            
            datatrans2 = DataTrans(NavStep.TURN_TO) .set_name("datatrans2")
            datatrans2 .add_sources(dispatch) .add_destinations(turn_to)
            
            datatrans3 = DataTrans(NavStep.DOORPASS) .set_name("datatrans3")
            datatrans3 .add_sources(dispatch) .add_destinations(doorpass)
            
            datatrans4 = DataTrans(NavStep.BACKUP) .set_name("datatrans4")
            datatrans4 .add_sources(dispatch) .add_destinations(backup)
            
            completiontrans5 = CompletionTrans() .set_name("completiontrans5")
            completiontrans5 .add_sources(drive) .add_destinations(next)
            
            failuretrans2 = FailureTrans() .set_name("failuretrans2")
            failuretrans2 .add_sources(drive) .add_destinations(parentfails2)
            
            completiontrans6 = CompletionTrans() .set_name("completiontrans6")
            completiontrans6 .add_sources(turn_to) .add_destinations(next)
            
            failuretrans3 = FailureTrans() .set_name("failuretrans3")
            failuretrans3 .add_sources(turn_to) .add_destinations(parentfails3)
            
            completiontrans7 = CompletionTrans() .set_name("completiontrans7")
            completiontrans7 .add_sources(doorpass) .add_destinations(next)
            
            failuretrans4 = FailureTrans() .set_name("failuretrans4")
            failuretrans4 .add_sources(doorpass) .add_destinations(parentfails4)
            
            completiontrans8 = CompletionTrans() .set_name("completiontrans8")
            completiontrans8 .add_sources(backup) .add_destinations(next)
            
            failuretrans5 = FailureTrans() .set_name("failuretrans5")
            failuretrans5 .add_sources(backup) .add_destinations(parentfails5)
            
            successtrans2 = SuccessTrans() .set_name("successtrans2")
            successtrans2 .add_sources(next) .add_destinations(dispatch)
            
            completiontrans9 = CompletionTrans() .set_name("completiontrans9")
            completiontrans9 .add_sources(next) .add_destinations(parentcompletes3)
            
            return self

        # End of PilotExecutePlan
    # End of PilotBase


#---------------- PilotToObject ----------------

class PilotToObject(PilotBase):
    "Use the wavefront planner to navigate to a distant object."
    def __init__(self, object_spec=None, execute=True):
        super().__init__()
        self.object_spec = object_spec
        self.execute = execute

    def start(self, event=None):
        if isinstance(event,DataEvent):
            if isinstance(event.data, WorldObject):
                self.object = event.data
                self.object_spec = None
            elif isinstance(event.data, str) and event.data in self.robot.world_map.objects:
                self.object_spec = event.data
            else:
                raise ValueError('DataEvent to PilotToObject must be a WorldObject or id string', event)
        if isinstance(self.object_spec, WorldObject):
            self.object = self.object_spec
        elif isinstance(self.object_spec, str):
            self.object = self.robot.world_map.objects[self.object_spec]
        if not self.object:
            raise ValueError('PilotToObject has no target object specified')
        super().start(event)

    class OrientToObject(TurnToward):
        def start(self,event=None):
            self.object_spec = self.parent.object
            super().start(event)

    class CenterTheObject(Turn):
        DEGREES_PER_PIXEL = 0.0829
        def start(self, event=None):
            obj = self.parent.object
            if obj.is_visible:
                if hasattr(obj, 'spec'):   # aivision object
                    spec = self.parent.object.spec
                    pixel_error = self.robot.camera.center[0] - (spec['originx'] + spec['width']/2)*AIVISION_RESOLUTION_SCALE
                elif hasattr(obj, 'marker'): # aruco marker
                    center = np.mean(obj.marker.corners, axis=1)
                    pixel_error = self.robot.camera.center[0] - center[0,0]
                else:
                    pixel_error = 0
            else:
                print('*** PilotToObject: target object not currently visible: cannot center it')
                pixel_error = 0
            self.angle_deg = pixel_error * self.DEGREES_PER_PIXEL
            super().start(event)

    class CheckArrival(StateNode):
        def start(self, event=None):
            super().start(event)
            pose = self.robot.pose
            if True: # *** TODO: check if we've arrived at the target shape
                self.post_success()
            else:
                self.post_failure()

    def setup(self):
        #         # PilotToObject machine
        # 
        #         launch: self.ClearDisplays() =N=> self.SendObject() =D=> planner
        # 
        #         #planner: PathPlannerProcess() =D=> recv
        #         planner: PathPlanToObject() =D=> recv
        #         planner =C=> ParentCompletes() # already close enough
        #         planner =PILOT=> ParentPilotEvent() =N=> Print('Path planner failed')
        # 
        #         recv: self.ReceivePlan() =D=> exec
        # 
        #         exec: self.PilotExecutePlan()
        #         exec =C=> StateNode() =T(1)=> check
        #         exec =F=> ParentFails()
        # 
        #         check: self.CheckArrival()
        #         check =S=> orient
        #         check =F=> Print("NavPlan failed to reach goal: trying again") =N=> planner
        # 
        #         orient: self.OrientToObject() =C=> Print("Pilot OrientToObject completes") =T(0.5)=> center
        # 
        #         center: self.CenterTheObject() =C=> Print("Pilot CenterTheObject completes") =N=> ParentCompletes()
        
        # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
        
        launch = self.ClearDisplays() .set_name("launch") .set_parent(self)
        sendobject1 = self.SendObject() .set_name("sendobject1") .set_parent(self)
        planner = PathPlanToObject() .set_name("planner") .set_parent(self)
        parentcompletes4 = ParentCompletes() .set_name("parentcompletes4") .set_parent(self)
        parentpilotevent1 = ParentPilotEvent() .set_name("parentpilotevent1") .set_parent(self)
        print1 = Print('Path planner failed') .set_name("print1") .set_parent(self)
        recv = self.ReceivePlan() .set_name("recv") .set_parent(self)
        exec = self.PilotExecutePlan() .set_name("exec") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        parentfails6 = ParentFails() .set_name("parentfails6") .set_parent(self)
        check = self.CheckArrival() .set_name("check") .set_parent(self)
        print2 = Print("NavPlan failed to reach goal: trying again") .set_name("print2") .set_parent(self)
        orient = self.OrientToObject() .set_name("orient") .set_parent(self)
        print3 = Print("Pilot OrientToObject completes") .set_name("print3") .set_parent(self)
        center = self.CenterTheObject() .set_name("center") .set_parent(self)
        print4 = Print("Pilot CenterTheObject completes") .set_name("print4") .set_parent(self)
        parentcompletes5 = ParentCompletes() .set_name("parentcompletes5") .set_parent(self)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(launch) .add_destinations(sendobject1)
        
        datatrans5 = DataTrans() .set_name("datatrans5")
        datatrans5 .add_sources(sendobject1) .add_destinations(planner)
        
        datatrans6 = DataTrans() .set_name("datatrans6")
        datatrans6 .add_sources(planner) .add_destinations(recv)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(planner) .add_destinations(parentcompletes4)
        
        pilottrans1 = PilotTrans() .set_name("pilottrans1")
        pilottrans1 .add_sources(planner) .add_destinations(parentpilotevent1)
        
        nulltrans2 = NullTrans() .set_name("nulltrans2")
        nulltrans2 .add_sources(parentpilotevent1) .add_destinations(print1)
        
        datatrans7 = DataTrans() .set_name("datatrans7")
        datatrans7 .add_sources(recv) .add_destinations(exec)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(exec) .add_destinations(statenode1)
        
        timertrans1 = TimerTrans(1) .set_name("timertrans1")
        timertrans1 .add_sources(statenode1) .add_destinations(check)
        
        failuretrans6 = FailureTrans() .set_name("failuretrans6")
        failuretrans6 .add_sources(exec) .add_destinations(parentfails6)
        
        successtrans3 = SuccessTrans() .set_name("successtrans3")
        successtrans3 .add_sources(check) .add_destinations(orient)
        
        failuretrans7 = FailureTrans() .set_name("failuretrans7")
        failuretrans7 .add_sources(check) .add_destinations(print2)
        
        nulltrans3 = NullTrans() .set_name("nulltrans3")
        nulltrans3 .add_sources(print2) .add_destinations(planner)
        
        completiontrans12 = CompletionTrans() .set_name("completiontrans12")
        completiontrans12 .add_sources(orient) .add_destinations(print3)
        
        timertrans2 = TimerTrans(0.5) .set_name("timertrans2")
        timertrans2 .add_sources(print3) .add_destinations(center)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(center) .add_destinations(print4)
        
        nulltrans4 = NullTrans() .set_name("nulltrans4")
        nulltrans4 .add_sources(print4) .add_destinations(parentcompletes5)
        
        return self

#---------------- PilotToPose ----------------

class PilotToPose(PilotBase):
    "Use the rrt path planner for short-range navigation to a specific pose."
    def __init__(self, target_pose=None, verbose=False, max_iter=RRT.DEFAULT_MAX_ITER, execute=True):
        super().__init__()
        self.target_pose = target_pose
        self.target_object = None
        self.verbose = verbose
        self.max_iter = max_iter
        self.execute = execute

    def start(self, event=None):
        if isinstance(event, DataEvent):
            if isinstance(event.data, WorldObject):
                self.target_object = event.data
                self.target_pose = copy.copy(event.data.pose)
                self.target_pose.theta = nan
            elif isinstance(event.data, Pose):
                self.target_object = None
                self.target_pose = event.data
            else:
                raise ValueError('Not a world object or pose:', event.data)
        self.robot.rrt.max_iter = self.max_iter
        super().start(event)

    class PilotRRTPlanner(StateNode):
        MIN_DISTANCE_THRESHOLD = 10 # mm

        def start(self,event=None):
            super().start(event)
            tpose = self.parent.target_pose
            if not isinstance(tpose,Pose) or (tpose.x == 0 and tpose.y == 0 and tpose.theta == 0):
                print("Pilot: target pose is invalid: %s" % tpose)
                self.parent.post_event(PilotEvent(InvalidPose, pose=tpose))
                self.parent.post_failure()
                return
            if self.robot.particle_filter.state != ParticleFilter.LOCALIZED:
                print('PilotRRTPlanner: Robot not localized!')
                self.parent.post_event(PilotEvent(NotLocalized))
                self.parent.post_failure()
                return
            pose = self.robot.pose
            distance = sqrt((tpose.x - pose.x)**2 + (tpose.y - pose.y)**2)
            if distance <= self.MIN_DISTANCE_THRESHOLD:
                print(f'PilotRRTPlanner: distance {distance:.1f} mm already close enough')
                self.post_completion()
                return
            else:
                print(f'PilotRRTPlanner: {distance = :.1f} mm')
            start_node = RRTNode(x=pose.x, y=pose.y, q=pose.theta)
            goal_node = RRTNode(x=tpose.x, y=tpose.y, q=tpose.theta)

            start_escape_move = None
            try:
                (treeA, treeB, path) = self.rrt_plan(start_node, goal_node)

            except StartCollides as e:
                # See if we can escape the start collision using canned headings.
                # This could be made more sophisticated, e.g., using arcs.
                #print('planner',e,'start',start_node)
                escape_distance = 50 # mm
                escape_headings = (0, +30/180.0*pi, -30/180.0*pi, pi, pi/2, -pi/2)
                for phi in escape_headings:
                    if phi != pi:
                        new_q = wrap_angle(start_node.q + phi)
                        d = escape_distance
                    else:
                        new_q = start_node.q
                        d = -escape_distance
                    new_start = RRTNode(x=start_node.x + d*cos(new_q),
                                        y=start_node.y + d*sin(new_q),
                                        q=new_q)
                    # print('trying start escape', new_start)
                    if not self.robot.rrt.collides(new_start):
                        start_escape_move = (phi, start_node, new_start)
                        start_node = new_start
                        break
                if start_escape_move is None:
                    print('PilotRRTPlanner: Start collides!',e)
                    self.parent.post_event(PilotEvent(StartCollides, args=e.args))
                    self.parent.post_failure()
                    return
                try:
                    (treeA, treeB, path) = self.rrt_plan(start_node, goal_node)
                except GoalCollides as e:
                    print('PilotRRTPlanner: Goal collides!',e)
                    self.parent.post_event(PilotEvent(GoalCollides, args=e.args))
                    self.parent.post_failure()
                    return
                except MaxIterations as e:
                    print('PilotRRTPlanner: Max iterations %d exceeded!' % e.args[0])
                    self.parent.post_event(PilotEvent(MaxIterations, args=e.args))
                    self.parent.post_failure()
                    return
                #print('replan',path)

            except GoalCollides as e:
                print('PilotRRTPlanner: Goal collides!',e)
                self.parent.post_event(PilotEvent(GoalCollides, args=e.args))
                self.parent.post_failure()
                return
            except MaxIterations as e:
                print('PilotRRTPlanner: Max iterations %d exceeded!' % e.args[0])
                self.parent.post_event(PilotEvent(MaxIterations, args=e.args))
                self.parent.post_failure()
                return

            if self.parent.verbose:
                print('Path planner generated',len(treeA)+len(treeB),'nodes.')
            if self.parent.robot.path_viewer:
                self.parent.robot.path_viewer.add_tree(path, (1, 0, 0, 0.75))

            self.robot.rrt.draw_path = path

            # Construct the nav plan
            if self.parent.verbose:
                [print(' ',x) for x in path]

            doors = self.robot.world_map.generate_doorway_list()
            navplan = PathPlanner.from_path(path, doors)
            print('navplan=',navplan, '   steps=',navplan.steps)

            # Insert the StartCollides escape move if there is one
            if start_escape_move:
                phi, start, new_start = start_escape_move
                if phi == pi:
                    escape_step = NavStep(NavStep.BACKUP, [new_start])
                    navplan.steps.insert(0, escape_step)
                elif navplan.steps[0].type == NavStep.DRIVE:
                    # Insert at the beginning the original start node we replaced with new_start
                    navplan.steps[0].param.insert(0, start_node)
                else:
                    # Shouldn't get here, but just in case
                    escape_step = NavStep(NavStep.DRIVE, (RRTNode(start.x,start.y), RRTNode(new_start.x,new_start.y)))
                    navplan.steps.insert(0, escape_step)

            #print('finalnavplan steps:', navplan.steps)

            # If no doorpass, we're good to go
            last_step = navplan.steps[-1]
            grid_display = None
            if last_step.type != NavStep.DOORPASS:
                self.post_data((navplan,grid_display))
                return

            # We planned for a doorpass as the last step; replan to the outer gate.
            door = last_step.param
            last_node = navplan.steps[-2].param[-1]
            gate, side = DoorPass.calculate_gate((last_node.x, last_node.y), door, DoorPass.OUTER_GATE_DISTANCE)
            goal_node = RRTNode(x=gate[0], y=gate[1], q=gate[2])
            print('new goal is', goal_node)
            try:
                (_, _, path) = self.rrt_plan(start_node, goal_node)
            except Exception as e:
                print('Pilot replanning for door gateway failed!', e.args)
            navplan = PathPlanner.from_path(path, [])
            navplan.steps.append(last_step)  # Add the doorpass step
            self.post_data((navplan,grid_display))

        def rrt_plan(self,start_node,goal_node):
            self.robot.rrt.generate_obstacles(self.parent.target_object,
                                              PathPlanner.skinny_obstacle_inflation,
                                              PathPlanner.skinny_wall_inflation,
                                              PathPlanner.skinny_doorway_adjustment)
            return self.robot.rrt.plan_path(start_node, goal_node, goal_object=self.parent.target_object)

        # ----- End of PilotRRTPlanner -----

    class CheckArrival(StateNode):
        def start(self, event=None):
            super().start(event)
            pose_diff = self.robot.pose - self.parent.target_pose
            distance = sqrt(pose_diff.x**2 + pose_diff.y**2)
            print(f'{distance=}  {pose_diff=}')
            MAX_TARGET_DISTANCE = 50.0 # mm
            if distance <= MAX_TARGET_DISTANCE:
                self.post_success()
            else:
                self.post_failure()


    def setup(self):
        #         # PilotToPose machine
        # 
        #         launch: self.ClearDisplays() =N=> planner
        # 
        #         planner: self.PilotRRTPlanner() =D=> recv
        #         planner =C=> ParentCompletes()  # already close enough
        #         planner =PILOT=> ParentPilotEvent() =N=> Print('Path planner failed')
        # 
        #         recv: self.ReceivePlan() =D=> exec
        # 
        #         exec: self.PilotExecutePlan()
        #         exec =C=> check
        #         exec =F=> ParentFails()
        # 
        #         check: self.CheckArrival()
        #         check =S=> ParentCompletes()
        #         check =F=> planner
        
        # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
        
        launch = self.ClearDisplays() .set_name("launch") .set_parent(self)
        planner = self.PilotRRTPlanner() .set_name("planner") .set_parent(self)
        parentcompletes6 = ParentCompletes() .set_name("parentcompletes6") .set_parent(self)
        parentpilotevent2 = ParentPilotEvent() .set_name("parentpilotevent2") .set_parent(self)
        print5 = Print('Path planner failed') .set_name("print5") .set_parent(self)
        recv = self.ReceivePlan() .set_name("recv") .set_parent(self)
        exec = self.PilotExecutePlan() .set_name("exec") .set_parent(self)
        parentfails7 = ParentFails() .set_name("parentfails7") .set_parent(self)
        check = self.CheckArrival() .set_name("check") .set_parent(self)
        parentcompletes7 = ParentCompletes() .set_name("parentcompletes7") .set_parent(self)
        
        nulltrans5 = NullTrans() .set_name("nulltrans5")
        nulltrans5 .add_sources(launch) .add_destinations(planner)
        
        datatrans8 = DataTrans() .set_name("datatrans8")
        datatrans8 .add_sources(planner) .add_destinations(recv)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(planner) .add_destinations(parentcompletes6)
        
        pilottrans2 = PilotTrans() .set_name("pilottrans2")
        pilottrans2 .add_sources(planner) .add_destinations(parentpilotevent2)
        
        nulltrans6 = NullTrans() .set_name("nulltrans6")
        nulltrans6 .add_sources(parentpilotevent2) .add_destinations(print5)
        
        datatrans9 = DataTrans() .set_name("datatrans9")
        datatrans9 .add_sources(recv) .add_destinations(exec)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(exec) .add_destinations(check)
        
        failuretrans8 = FailureTrans() .set_name("failuretrans8")
        failuretrans8 .add_sources(exec) .add_destinations(parentfails7)
        
        successtrans4 = SuccessTrans() .set_name("successtrans4")
        successtrans4 .add_sources(check) .add_destinations(parentcompletes7)
        
        failuretrans9 = FailureTrans() .set_name("failuretrans9")
        failuretrans9 .add_sources(check) .add_destinations(planner)
        
        return self


class PilotPushToPose(PilotToPose):
    def __init__(self,pose):
        super().__init__(pose)
        self.max_turn = 20*(pi/180)

    def rrt_plan(self,start_node,goal_node):
        self.robot.rrt.step_size=20
        return self.robot.rrt.plan_push_chip(start_node,goal_node)


class PilotFrustration(StateNode):

    def __init__(self, text_template=None):
        super().__init__()
        self.text_template = text_template  # contains at most one '%s'

    class SayObject(Say):
        def start(self, event=None):
            text_template = self.parent.text_template
            try:
                object_name = self.parent.parent.object.name   # for rooms
            except:
                try:
                    object_name = self.parent.parent.object.id   # for cubes
                except:
                    object_name = None
            if text_template is not None:
                if '%' in text_template:
                    self.text = text_template % object_name
                else:
                    self.text = text_template
            elif object_name is not None:
                self.text = 'Can\'t reach %s' % object_name
            else:
                self.text = 'stuck'
            self.robot.rrt.text = self.text
            super().start(event)


    def setup(self):
        #         launcher: AbortAllActions() =N=> {speak, turn}
        # 
        #         speak: self.SayObject()
        # 
        #         turn: StateNode() =RND=> {left, right}
        #         left: Turn(5) =C=> left2: Turn(-5)
        #         right: Turn(-5) =C=> right2: Turn(5)
        # 
        #         {speak, left2, right2} =C(2)=> done
        # 
        #         done: ParentCompletes()
        
        # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
        
        launcher = AbortAllActions() .set_name("launcher") .set_parent(self)
        speak = self.SayObject() .set_name("speak") .set_parent(self)
        turn = StateNode() .set_name("turn") .set_parent(self)
        left = Turn(5) .set_name("left") .set_parent(self)
        left2 = Turn(-5) .set_name("left2") .set_parent(self)
        right = Turn(-5) .set_name("right") .set_parent(self)
        right2 = Turn(5) .set_name("right2") .set_parent(self)
        done = ParentCompletes() .set_name("done") .set_parent(self)
        
        nulltrans7 = NullTrans() .set_name("nulltrans7")
        nulltrans7 .add_sources(launcher) .add_destinations(speak,turn)
        
        randomtrans1 = RandomTrans() .set_name("randomtrans1")
        randomtrans1 .add_sources(turn) .add_destinations(left,right)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(left) .add_destinations(left2)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(right) .add_destinations(right2)
        
        completiontrans18 = CompletionTrans(2) .set_name("completiontrans18")
        completiontrans18 .add_sources(speak,left2,right2) .add_destinations(done)
        
        return self

# ================ DoorPass ================

pilot_global_doorpass_node = "Will be filled in by simple_cli due to circularity"

class DoorPass(StateNode):
    """Pass through a doorway. Assumes the doorway is nearby and unobstructed."""

    OUTER_GATE_DISTANCE = 150 # mm
    INNER_GATE_DISTANCE =  70 # mm -- currently unused

    def __init__(self, door_spec=None):
        self.door_spec = door_spec
        super().__init__()

    def start(self, event=None):
        door_spec = self.door_spec
        if isinstance(event,DataEvent):
            door_spec = event.data
        if isinstance(door_spec, int):
            door_spec ='Doorway-%d:0.a' % door_spec
        if isinstance(door_spec, str):
            doorobj = self.robot.world_map.objects.get(door_spec)
        elif isinstance(door_spec, DoorwayObj):
            doorobj = door_spec
        else:
            doorobj = None
        if isinstance(doorobj, DoorwayObj):
            self.object = doorobj
        else:
            print("Error in DoorPass: no doorway named %s" % repr(door_spec))
            self.punt_super_start()
            self.post_failure()
            return
        super().start(event)


    @staticmethod
    def calculate_gate(start_point, door, offset):
        """Returns closest gate point (gx, gy)"""
        (rx,ry) = start_point
        dx = door.pose.x
        dy = door.pose.y
        dtheta = door.pose.theta
        # calculate gates on either side of the door, then pick the closest
        pt1x = dx + offset * cos(dtheta)
        pt1y = dy + offset * sin(dtheta)
        pt2x = dx + offset * cos(dtheta+pi)
        pt2y = dy + offset * sin(dtheta+pi)
        dist1sq = (pt1x-rx)**2 + (pt1y-ry)**2
        dist2sq = (pt2x-rx)**2 + (pt2y-ry)**2
        if dist1sq < dist2sq:
            side = +1  # we're on the front side of the wall
            return (pt1x, pt1y, wrap_angle(dtheta+pi)), side
        else:
            side = -1  # we're on the back side of the wall
            return (pt2x, pt2y, dtheta), side

    class AwayFromCollide(Forward):
        def start(self, event=None):
            if isinstance(event,DataEvent):
                startNode = event.data[0]
                collideObj = event.data[1]
                pose = self.robot.pose
                (rx, ry, rtheta) = pose.x, pose.y, pose.theta
                cx, cy = collideObj.center[0,0],collideObj.center[1,0]
                ctheta = atan2(cy-ry, cx-rx)
                delta_angle = wrap_angle(ctheta - rtheta)
                delta_angle = delta_angle/pi*180
                if -90 < delta_angle and delta_angle < 90:
                    self.distance_mm = -40
                else:
                    self.distance_mm = 40
                self.drive_speed = 50
                super().start(event)
            else:
                raise ValueError('DataEvent to AwayFromCollide must be a StartCollides.args', event.data)
                super().start(event)
                self.post_failure()

    class TravelToGate(PilotToPose):
        def __init__(self,offset):
            self.offset = offset
            super().__init__()

        def start(self,event=None):
            pose = self.robot.pose
            (rx, ry, rtheta) = pose.x, pose.y, pose.theta
            (gate_x, gate_y, _), side = DoorPass.calculate_gate((rx,ry), self.parent.object, offset=self.offset)
            self.parent.side = side
            dpose = self.parent.object.pose
            target_theta = dpose.theta if side == -1 else dpose.theta + pi
            self.target_pose = Pose(gate_x, gate_y, theta=target_theta)
            super().start()

    class TurnToGate(Turn):
        """Turn to the approach gate, or post success if we're already there."""
        def __init__(self,offset):
            self.offset = offset
            super().__init__(turn_speed=45)

        def start(self,event=None):
            pose = self.robot.pose
            (rx, ry, rtheta) = pose.x, pose.y, pose.theta
            (gate_x, gate_y, _), side = DoorPass.calculate_gate((rx,ry), self.parent.object, offset=self.offset)
            bearing = atan2(gate_y-ry, gate_x-rx)
            turn = wrap_angle(bearing - rtheta)
            print('^^ TurnToGate: gate=(%.1f, %.1f)  offset=%.1f rtheta=%.1f  bearing=%.1f  turn=%.1f' %
                  (gate_x, gate_y, self.offset, rtheta*180/pi, bearing*180/pi, turn*180/pi))
            MIN_TURN_ANGLE = 2.5 * (pi/180)
            if abs(turn) < MIN_TURN_ANGLE:
                self.angle_deg = 0
                super().start(event)
                self.post_completion()
            else:
                self.angle_deg = turn * 180/pi
                super().start(event)


    class ForwardToGate(Forward):
        """Travel forward to reach the approach gate."""
        def __init__(self,offset):
            self.offset = offset
            super().__init__()

        def start(self,event=None):
            rx = self.robot.pose.x
            ry = self.robot.pose.y
            (gate_x, gate_y, _), side = DoorPass.calculate_gate((rx,ry), self.parent.object, offset=self.offset)
            self.distance_mm = sqrt((gate_x-rx)**2 + (gate_y-ry)**2)
            self.drive_speed = 50
            super().start(event)

    class TurnToFaceWall(Turn):
        def start(self,event=None):
            robot_theta = self.robot.pose.theta
            if self.parent.side == +1:
                door_theta = self.parent.object.pose.theta +pi
            else:
                door_theta = self.parent.object.pose.theta
            self.angle_deg = wrap_angle(door_theta - robot_theta) * 180/pi
            super().start(event)

    class CenterOnDoorway(Sideways):
        def start(self, event=None):
            door = self.parent.object
            index = door.index
            wall = door.wall
            spec = wall.wall_spec
            door_x = spec.doorways[index]['x']
            side = self.parent.side
            markers = [(num, spec['x']) for (num,spec) in spec.marker_specs.items()
                       if spec['side'] == side]
            lefts = [m for m in markers if m[1] < door_x]
            left = max(lefts, key=lambda p : p[1])
            rights = [m for m in markers if m[1] > door_x]
            right = min(rights, key=lambda p: p[1])
            print(f'left={lefts} left={left} rights={rights} right={right} door_x={door_x}')
            seen_markers = self.robot.aruco_detector.seen_marker_objects
            left_marker = seen_markers.get(left[0], None)
            right_marker = seen_markers.get(right[0], None)
            left_offset = left_marker.camera_coords[0] - (door_x - left[1]) if left_marker else None
            right_offset = right_marker.camera_coords[0] - (door_x - right[1]) if right_marker else None
            if left_offset and right_offset:
                self.distance_mm = (left_offset + right_offset) / 2
            elif left_offset:
                print('DoorPass: right marker not visible!')
                self.distance_mm = left_offset
            elif right_offset:
                print('DoorPass: left marker not visible!')
                self.distance_mm = right_offset
            else:
                print('DoorPass: markers not visible!')
                self.distance_mm = 0
            print(f'offset = {self.distance_mm}')
            super().start(event)

    class CenterOnDoorway_Old(Sideways):
        def start(self,event=None):
            """Calculate the vector from the robot to the doorway and rotate it into the robot's
            frame so the y component gives the sideways distance to line up with the door."""
            # NOTE: since the robot won't be perfectly facing the door we should perhaps
            # adjust theta to reflect that.
            door_pose = self.parent.object.pose
            robot_pose = self.robot.pose
            dx = door_pose.x - robot_pose.x
            dy = door_pose.y - robot_pose.y
            vec = aboutZ(robot_pose.theta).dot(point(dx,dy))
            self.distance_mm = vec[1,0]
            print(f'dpose={door_pose} rpose={robot_pose} dx={dx:.1f} dy={dy:.1f}' +
                  f' vec={vec[0,0]:.1f},{vec[1,0]:.1f} dist={self.distance_mm:.1f}')
            super().start(event)
        

    class TurnToMarker(Turn):
        """Use camera image and native pose to center the door marker."""
        def start(self,event=None):
            marker_ids = self.parent.object.marker_ids
            marker = self.robot.aruco_detector.seen_marker_objects.get(marker_ids[0], None) or \
                     self.robot.aruco_detector.seen_marker_objects.get(marker_ids[1], None)
            if not marker:
                self.angle_deg = 0
                super().start(event)
                print("TurnToMarker failed to find marker %s or %s!" % marker_ids)
                self.post_failure()
                return
            else:
                print('TurnToMarker saw marker', marker)
            sensor_dist = marker.camera_distance
            sensor_bearing = atan2(marker.camera_coords[0],
                                   marker.camera_coords[2])
            x = self.robot.pose.position.x
            y = self.robot.pose.position.y
            theta = self.robot.pose.rotation.angle_z.radians
            direction = theta + sensor_bearing
            dx = sensor_dist * cos(direction)
            dy = sensor_dist * sin(direction)
            turn = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians)
            if abs(turn) < 0.5*pi/180:
                self.angle_deg = 0
            else:
                self.angle_deg = turn * 180/pi
            print("TurnToMarker %s turning by %.1f degrees" % (self.name, self.angle.degrees))
            super().start(event)

    class DriveThrough(Forward):
        """Travel forward to drive through the gate."""
        def __init__(self):
            super().__init__()

        def start(self,event=None):
            pose = self.robot.pose
            (rx, ry, rtheta) = pose.x, pose.y, pose.theta
            (gate_x, gate_y, gate_theta), side = DoorPass.calculate_gate((rx,ry), self.parent.object, offset=5)
            dist = sqrt((gate_x-rx)**2 + (gate_y-ry)**2)
            offset = 120
            delta_theta = wrap_angle(rtheta-(gate_theta+pi/2))
            delta_dist = abs(offset/sin(delta_theta))
            dist += delta_dist
            self.distance_mm = dist
            self.drive_speed = 50
            super().start(event)

    def setup(self):
        #         # DoorPass machine
        # 
        #         check_start: PilotCheckStartDetail()
        #         check_start =S=> travel_to_gate1 # turn_to_gate1
        #         check_start =D=> away_from_collide
        #         check_start =F=> Forward(-80) =C=> check_start2
        # 
        #         travel_to_gate1: self.TravelToGate(DoorPass.OUTER_GATE_DISTANCE)
        #         travel_to_gate1 =C=> travelwait: StateNode() =traveltimer:T(1)=> align_with_wall
        #         travel_to_gate1 =PILOT=> ParentFails()        
        # 
        #         align_with_wall: self.TurnToFaceWall() =C=>
        #             Print('facing the wall') =T(1)=> center_on_doorway
        # 
        #         center_on_doorway: self.CenterOnDoorway() =C=>
        #             Print('centered on doorway') =C=> through_door
        # 
        #         through_door: self.DriveThrough() =C=> ParentCompletes()
        # 
        #         away_from_collide: self.AwayFromCollide() =C=> StateNode() =T(0.2)=> check_start2
        #         away_from_collide =F=> travel_to_gate1
        # 
        #         check_start2: PilotCheckStartDetail()
        #         check_start2 =S=> travel_to_gate1
        #         check_start2 =D=> away_from_collide2
        #         check_start2 =F=> ParentFails()
        # 
        #         away_from_collide2: self.AwayFromCollide() =C=> StateNode() =T(0.2)=> check_start3
        #         away_from_collide2 =F=> check_start3
        # 
        #         check_start3: PilotCheckStart()
        #         check_start3 =S=> travel_to_gate1
        #         check_start3 =F=> ParentFails()
        # 
        
        # Code generated by genfsm on Fri Feb 20 05:26:12 2026:
        
        check_start = PilotCheckStartDetail() .set_name("check_start") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        travel_to_gate1 = self.TravelToGate(DoorPass.OUTER_GATE_DISTANCE) .set_name("travel_to_gate1") .set_parent(self)
        travelwait = StateNode() .set_name("travelwait") .set_parent(self)
        parentfails8 = ParentFails() .set_name("parentfails8") .set_parent(self)
        align_with_wall = self.TurnToFaceWall() .set_name("align_with_wall") .set_parent(self)
        print6 = Print('facing the wall') .set_name("print6") .set_parent(self)
        center_on_doorway = self.CenterOnDoorway() .set_name("center_on_doorway") .set_parent(self)
        print7 = Print('centered on doorway') .set_name("print7") .set_parent(self)
        through_door = self.DriveThrough() .set_name("through_door") .set_parent(self)
        parentcompletes8 = ParentCompletes() .set_name("parentcompletes8") .set_parent(self)
        away_from_collide = self.AwayFromCollide() .set_name("away_from_collide") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        check_start2 = PilotCheckStartDetail() .set_name("check_start2") .set_parent(self)
        parentfails9 = ParentFails() .set_name("parentfails9") .set_parent(self)
        away_from_collide2 = self.AwayFromCollide() .set_name("away_from_collide2") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        check_start3 = PilotCheckStart() .set_name("check_start3") .set_parent(self)
        parentfails10 = ParentFails() .set_name("parentfails10") .set_parent(self)
        
        successtrans5 = SuccessTrans() .set_name("successtrans5")
        successtrans5 .add_sources(check_start) .add_destinations(travel_to_gate1)
        
        datatrans10 = DataTrans() .set_name("datatrans10")
        datatrans10 .add_sources(check_start) .add_destinations(away_from_collide)
        
        failuretrans10 = FailureTrans() .set_name("failuretrans10")
        failuretrans10 .add_sources(check_start) .add_destinations(forward1)
        
        completiontrans19 = CompletionTrans() .set_name("completiontrans19")
        completiontrans19 .add_sources(forward1) .add_destinations(check_start2)
        
        completiontrans20 = CompletionTrans() .set_name("completiontrans20")
        completiontrans20 .add_sources(travel_to_gate1) .add_destinations(travelwait)
        
        traveltimer = TimerTrans(1) .set_name("traveltimer")
        traveltimer .add_sources(travelwait) .add_destinations(align_with_wall)
        
        pilottrans3 = PilotTrans() .set_name("pilottrans3")
        pilottrans3 .add_sources(travel_to_gate1) .add_destinations(parentfails8)
        
        completiontrans21 = CompletionTrans() .set_name("completiontrans21")
        completiontrans21 .add_sources(align_with_wall) .add_destinations(print6)
        
        timertrans3 = TimerTrans(1) .set_name("timertrans3")
        timertrans3 .add_sources(print6) .add_destinations(center_on_doorway)
        
        completiontrans22 = CompletionTrans() .set_name("completiontrans22")
        completiontrans22 .add_sources(center_on_doorway) .add_destinations(print7)
        
        completiontrans23 = CompletionTrans() .set_name("completiontrans23")
        completiontrans23 .add_sources(print7) .add_destinations(through_door)
        
        completiontrans24 = CompletionTrans() .set_name("completiontrans24")
        completiontrans24 .add_sources(through_door) .add_destinations(parentcompletes8)
        
        completiontrans25 = CompletionTrans() .set_name("completiontrans25")
        completiontrans25 .add_sources(away_from_collide) .add_destinations(statenode2)
        
        timertrans4 = TimerTrans(0.2) .set_name("timertrans4")
        timertrans4 .add_sources(statenode2) .add_destinations(check_start2)
        
        failuretrans11 = FailureTrans() .set_name("failuretrans11")
        failuretrans11 .add_sources(away_from_collide) .add_destinations(travel_to_gate1)
        
        successtrans6 = SuccessTrans() .set_name("successtrans6")
        successtrans6 .add_sources(check_start2) .add_destinations(travel_to_gate1)
        
        datatrans11 = DataTrans() .set_name("datatrans11")
        datatrans11 .add_sources(check_start2) .add_destinations(away_from_collide2)
        
        failuretrans12 = FailureTrans() .set_name("failuretrans12")
        failuretrans12 .add_sources(check_start2) .add_destinations(parentfails9)
        
        completiontrans26 = CompletionTrans() .set_name("completiontrans26")
        completiontrans26 .add_sources(away_from_collide2) .add_destinations(statenode3)
        
        timertrans5 = TimerTrans(0.2) .set_name("timertrans5")
        timertrans5 .add_sources(statenode3) .add_destinations(check_start3)
        
        failuretrans13 = FailureTrans() .set_name("failuretrans13")
        failuretrans13 .add_sources(away_from_collide2) .add_destinations(check_start3)
        
        successtrans7 = SuccessTrans() .set_name("successtrans7")
        successtrans7 .add_sources(check_start3) .add_destinations(travel_to_gate1)
        
        failuretrans14 = FailureTrans() .set_name("failuretrans14")
        failuretrans14 .add_sources(check_start3) .add_destinations(parentfails10)
        
        return self
