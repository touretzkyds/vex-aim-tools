import re
from importlib import __import__, reload
try:
    from termcolor import cprint
except:
    def cprint(string,color=None):
        print(string)

import cv2
ARUCO_DICT_4x4_100 = cv2.aruco.DICT_4X4_100

global robot_for_loading

from .evbase import EventRouter
from .base import StateNode
from .cam_viewer import CamViewer
from .worldmap_viewer import WorldMapViewer
from .aruco import *
from .worldmap import WorldMap
#from .particle import *
#from .aim_kin import *
#from .particle_viewer import ParticleViewer
#from .rrt import RRT
#from .path_viewer import PathViewer
#from .speech import SpeechListener, Thesaurus
from . import opengl
#from . import custom_objs
#from .perched import *
#from .sharedmap import *

running_fsm = None

class StateMachineProgram(StateNode):
    def __init__(self,
                 #kine_class = CozmoKinematics,
                 cam_viewer = True,
                 worldmap_viewer = True,
                 force_annotation = False,   # set to True for annotation even without cam_viewer
                 annotate_sdk = True,        # include annotations for SDK's object detections
                 annotated_scale_factor = 1, # set to 1 to avoid cost of resizing images
                 viewer_crosshairs = False,  # set to True to draw viewer crosshairs

                 particle_filter = True,
                 #landmark_test = SLAMSensorModel.is_solo_aruco_landmark,
                 particle_viewer = False,
                 particle_viewer_scale = 1.0,

                 aruco = True,
                 arucolibname = ARUCO_DICT_4x4_100,
                 aruco_disabled_ids = (17, 37),
                 aruco_marker_size = ARUCO_MARKER_SIZE,

                 perched_cameras = False,

                 rrt = None,
                 path_viewer = False,

                 speech = False,
                 speech_debug = False,
                 #thesaurus = Thesaurus(),
                 ):
        super().__init__()
        self.name = self.__class__.__name__.lower()
        self.parent = None
        self.robot.robot0.set_pose(0,0,0)

        if not hasattr(self.robot, 'erouter'):
            self.robot.erouter = EventRouter()
            self.robot.erouter.robot = self.robot
            self.robot.erouter.start()
        else:
            self.robot.erouter.clear()

        #self.kine_class = kine_class

        self.cam_viewer = cam_viewer
        self.viewer = None
        self.annotate_sdk = annotate_sdk
        self.force_annotation = force_annotation
        self.annotated_scale_factor = annotated_scale_factor
        self.viewer_crosshairs = viewer_crosshairs

        #self.particle_filter = particle_filter
        #self.landmark_test = landmark_test
        self.particle_viewer = particle_viewer
        self.particle_viewer_scale = particle_viewer_scale
        #self.picked_up_callback = self.robot_picked_up
        #self.put_down_handler = self.robot_put_down

        self.aruco = aruco
        self.aruco_marker_size = aruco_marker_size
        if self.aruco:
            self.robot.aruco = \
                Aruco(self.robot, arucolibname, aruco_marker_size, aruco_disabled_ids)

        self.perched_cameras = perched_cameras
        if self.perched_cameras:
            self.robot.perched = PerchedCameraThread(self.robot)

        self.robot.aruco_id = -1
        self.robot.use_shared_map = False
        #self.robot.world_map.server = ServerThread(self.robot)
        #self.robot.world_map.client = ClientThread(self.robot)
        #self.robot.world_map.is_server = True # Writes directly into perched.camera_pool

        self.worldmap_viewer = worldmap_viewer

        self.rrt = rrt
        self.path_viewer = path_viewer

        self.speech = speech
        self.speech_debug = speech_debug
        #self.thesaurus = thesaurus

    def start(self):
        global running_fsm
        running_fsm = self
        # Create a particle filter
        #if not isinstance(self.particle_filter,ParticleFilter):
        #    self.particle_filter = SLAMParticleFilter(self.robot, landmark_test=self.landmark_test)
        # elif isinstance(self.particle_filter,SLAMParticleFilter):
        #    self.particle_filter.clear_landmarks()
        #pf = self.particle_filter
        #self.robot.world.particle_filter = pf

        # Set up kinematics
        #self.robot.kine = self.kine_class(self.robot)
        self.robot.was_picked_up = False
        self.robot.carrying = None
        self.robot.fetching = None

        # robot.is_picked_up uses just the cliff detector, and can be fooled.
        # robot.pose.rotation does not encode pitch or roll, only yaw.
        # So use accelerometer data as our backup method.
        self.robot.really_picked_up = \
            (lambda robot :
             (lambda :
              robot.is_picked_up
              or (not robot.is_moving
                  and (robot.accelerometer.z < 5000
                       or robot.accelerometer.z > 13000))))(self.robot)
#                  and (robot.accelerometer.z < 5000
#                       or robot.accelerometer.z > 10300))))(self.robot)

        # World map and path planner
        #self.robot.world.rrt = self.rrt or RRT(self.robot)

        # Polling
        self.set_polling_interval(0.025)  # for kine and motion model update

        # Launch viewers
        if self.cam_viewer:
            if self.cam_viewer is True:
                self.cam_viewer = CamViewer(self.robot)
            self.cam_viewer.start()
        self.robot.cam_viewer = self.cam_viewer

        if self.worldmap_viewer:
            if self.worldmap_viewer is True:
                self.worldmap_viewer = WorldMapViewer(self.robot)
            self.worldmap_viewer.start()
        self.robot.worldmap_viewer = self.worldmap_viewer

        if self.particle_viewer:
            if self.particle_viewer is True:
                self.particle_viewer = \
                    ParticleViewer(self.robot, scale=self.particle_viewer_scale)
            self.particle_viewer.start()
        self.robot.world_map.particle_viewer = self.particle_viewer

        if self.path_viewer:
            if self.path_viewer is True:
                self.path_viewer = PathViewer(self.robot, self.robot.world.rrt)
            else:
                self.path_viewer.set_rrt(self.robot.world.rrt)
            self.path_viewer.start()
        self.robot.world_map.path_viewer = self.path_viewer

        # Start speech recognition if requested
        if self.speech:
            self.speech_listener = SpeechListener(self.robot,self.thesaurus,debug=self.speech_debug)
            self.speech_listener.start()

        # Call parent's start() to launch the state machine by invoking the start node.
        super().start()

    def stop(self):
        super().stop()
        self.robot.erouter.clear()

    def poll(self):
        # Invalidate cube pose if cube has been moving and isn't seen
        move_duration_regular_threshold = 0.5 # seconds
        move_duration_fetch_threshold = 1 # seconds

        # Update robot kinematic description
        #self.robot.kine.get_pose()

        # Handle robot being picked up or put down
        """
        if self.robot.really_picked_up():
            # robot is in the air
            if self.robot.was_picked_up:
                pass  # we already knew that
            else:
                self.picked_up_callback()
        else:  # robot is on the ground
            pf = self.robot.world.particle_filter
            if pf:
                if self.robot.was_picked_up:
                    self.put_down_handler()
                else:
                    pf.move()
        self.robot.was_picked_up = self.robot.really_picked_up()
        """

    def user_image(self,image,gray): pass

    def user_annotate(self,image):
        return image

    def process_image(self,image):
        # Aruco image processing
        if self.aruco:
            gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            self.robot.aruco.process_image(gray)
        # Other image processors can run here if the user supplies them.
        self.user_image(image,gray)
        # Done with image processing

        """
        # Annotate and display image if requested
        if self.force_annotation or self.viewer is not None:
            scale = self.annotated_scale_factor
                # Apply Cozmo SDK annotations and rescale.
                if self.annotate_sdk:
                    coz_ann = event.image.annotate_image(scale=scale)
                    annotated_im = numpy.array(coz_ann)
                elif scale != 1:
                    shape = curim.shape
                    dsize = (scale*shape[1], scale*shape[0])
                    annotated_im = cv2.resize(curim, dsize)
                else:
                    annotated_im = curim
                # Aruco annotation
                if self.aruco and \
                       len(self.robot.aruco.seen_marker_ids) > 0:
                    annotated_im = self.robot.world.aruco.annotate(annotated_im,scale)
                # Other annotators can run here if the user supplies them.
                annotated_im = self.user_annotate(annotated_im)
                # Done with annotation
                annotated_im = cv2.cvtColor(annotated_im,cv2.COLOR_RGB2BGR)

        # Use this heartbeat signal to look for new landmarks
        pf = self.robot.world.particle_filter
        if pf and not self.robot.really_picked_up():
            pf.look_for_new_landmarks()

        # Finally update the world map
        self.robot.world_map.update_map()
        """

################

def runfsm(module_name, running_modules=dict()):
    """runfsm('modname') reloads that module and expects it to contain
    a class of the same name. It calls that class's constructor and then
    calls the instance's start() method."""

    global running_fsm
    if running_fsm:
        running_fsm.stop()

    r_py = re.compile('.*\.py$')
    if r_py.match(module_name):
        print("\n'%s' is not a module name. Trying '%s' instead.\n" %
              (module_name, module_name[0:-3]))
        module_name = module_name[0:-3]

    found = False
    try:
        reload(running_modules[module_name])
        found = True
    except KeyError: pass
    except: raise
    if not found:
        try:
            running_modules[module_name] = __import__(module_name)
        except ImportError as e:
            print("Error loading %s: %s.  Check your search path.\n" %
                  (module_name,e))
            return
        except Exception as e:
            print('\n===> Error loading %s:' % module_name)
            raise

    py_filepath = running_modules[module_name].__file__
    fsm_filepath = py_filepath[0:-2] + 'fsm'
    try:
        py_time = datetime.datetime.fromtimestamp(os.path.getmtime(py_filepath))
        fsm_time = datetime.datetime.fromtimestamp(os.path.getmtime(fsm_filepath))
        if py_time < fsm_time:
            cprint('Warning: %s.py is older than %s.fsm. Should you run genfsm?' %
                   (module_name,module_name), color="yellow")
    except: pass

    # The parent node class's constructor must match the module name.
    the_module = running_modules[module_name]
    the_class = the_module.__getattribute__(module_name) \
                if module_name in dir(the_module) else None
    if isinstance(the_class,type) and issubclass(the_class,StateNode) and not issubclass(the_class,StateMachineProgram):
        cprint("%s is not an instance of StateMachineProgram.\n" % module_name, color="red")
        return
    if not isinstance(the_class,type) or not issubclass(the_class,StateMachineProgram):
        cprint("Module %s does not contain a StateMachineProgram named %s.\n" %
              (module_name, module_name), color="red")
        return
    the_module.robot = robot_for_loading
    #the_module.world = robot.world
    # Class's __init__ method will call setup, which can reference the above variables.
    running_fsm = the_class()
    robot_for_loading.loop.call_soon_threadsafe(running_fsm.start)
    return running_fsm

