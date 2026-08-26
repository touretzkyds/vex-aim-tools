from math import pi
import time
from typing import Any, Callable, Optional
import re
from importlib import __import__, reload
try:
    from termcolor import cprint
except:
    def cprint(string,color=None):
        print(string)

import cv2
import numpy as _np

import vex
from . import evbase

from .evbase import EventRouter
from .base import StateNode
from viewer.cam_viewer import CamViewer
from viewer.worldmap_viewer import WorldMapViewer
from .aruco import *
from .worldmap import WorldMap
from .wall_defs import default_wall_marker_dict
from .particle import *
from .utils import Pose
from viewer.particle_viewer import ParticleViewer
from .rrt import RRT
from viewer.path_viewer import PathViewer
from viewer.camera_overlay import apply_overlays
from viewer.lifecycle import ensure_viewer
from .camera import AIVISION_RESOLUTION_SCALE
from . import pilot
#from . import custom_objs
#from .perched import *
#from .sharedmap import *

running_fsm = None

class StateMachineProgram(StateNode):
    def __init__(self,
                 character_name = "Robot",
                 launch_cam_viewer = True,
                 launch_worldmap_viewer = True,
                 force_annotation = False,   # set to True for annotation even without cam_viewer
                 annotate_sdk = True,        # include annotations for SDK's object detections
                 annotated_scale_factor = 1, # set to 1 to avoid cost of resizing images
                 annotated_image_callback: Optional[Callable[[Any, dict], None]] = None,
                 viewer_crosshairs = False,  # set to True to draw viewer crosshairs
                 speech = True,
                 openai_model = None,        # set to 'gpt-5.6-sol' or other model name to override the client default
                 particle_filter = None,
                 num_particles = 500,
                 landmarks = dict(),
                 wall_marker_dict = default_wall_marker_dict,
                 launch_particle_viewer = False,
                 particle_viewer_scale = 1.0,
                 launch_path_viewer = False,
                 aruco = True,
                 dictionary_name = cv2.aruco.DICT_4X4_100,
                 aruco_disabled_ids = (17, 37),
                 aruco_marker_size = ARUCO_MARKER_SIZE,

                 perched_cameras = False,

                 rrt = None,
                 ):
        super().__init__()
        self.robot.character_name = character_name
        self.name = self.__class__.__name__.lower()
        self.parent = None
        self.robot.robot0.set_xy_position(0,0)
        self.robot.robot0.inertial.set_heading(0)
        if openai_model is not None:
            self.robot.openai_client.model = openai_model
        # print(f'OpenAI model: {self.robot.openai_client.model}')  # if wish to show the model in use

        if not hasattr(self.robot, 'erouter'):
            self.robot.erouter = EventRouter()
            self.robot.erouter.robot = self.robot
            self.robot.erouter.start()
        else:
            self.robot.erouter.clear()

        self.launch_cam_viewer = launch_cam_viewer
        self.viewer = None
        self.wall_marker_dict = wall_marker_dict or dict()
        self.annotate_sdk = annotate_sdk
        self.force_annotation = force_annotation
        self.annotated_scale_factor = annotated_scale_factor
        self.annotated_image_callback = annotated_image_callback
        self.viewer_crosshairs = viewer_crosshairs
        self.speech = speech
        self.launch_particle_viewer = launch_particle_viewer
        self.particle_viewer_scale = particle_viewer_scale
        self.launch_path_viewer = launch_path_viewer
        self.picked_up_handler = self.robot_picked_up_default
        self.put_down_handler = self.robot_put_down_default

        self.aruco = aruco
        self.aruco_marker_size = aruco_marker_size
        if self.aruco:
            self.robot.aruco_detector = \
                RobotArucoDetector(self.robot, dictionary_name, aruco_marker_size, aruco_disabled_ids)

        self.num_particles = num_particles
        self.landmarks = landmarks
        if isinstance(particle_filter, ParticleFilter):
            self.particle_filter = particle_filter
        elif particle_filter is None:
            self.particle_filter = \
                SLAMParticleFilter(self.robot, num_particles=self.num_particles,
                                   landmark_test=SLAMSensorModel.is_wall_landmark)
        elif particle_filter == False:
            self.particle_filter = None
        else:
            raise TypeError(f'Not a ParticleFilter instance: {particle_filter=}')

        self.perched_cameras = perched_cameras
        if self.perched_cameras:
            self.robot.perched = PerchedCameraThread(self.robot)

        self.robot.aruco_id = -1
        self.robot.use_shared_map = False
        #self.robot.world_map.server = ServerThread(self.robot)
        #self.robot.world_map.client = ClientThread(self.robot)
        #self.robot.world_map.is_server = True # Writes directly into perched.camera_pool

        self.launch_worldmap_viewer = launch_worldmap_viewer

    def start(self):
        global running_fsm
        running_fsm = self
        # Create a particle filter
        if self.particle_filter is None:
            self.particle_filter = SLAMParticleFilter(self.robot,
                                                      num_particles=self.num_particles,
                                                      landmarks=self.landmarks)
        elif isinstance(self.particle_filter,SLAMParticleFilter):
            self.particle_filter.clear_landmarks()
        self.robot.particle_filter = self.particle_filter

        # Set up robot state
        self.robot.was_picked_up = False
        self.robot.holding = None
        self.robot.fetching = None
        self.robot.robot0.led.on(vex.LightType.ALL_LEDS, vex.Color.TRANSPARENT)
        self.robot.clear_actuators()

        # World map
        self.robot.world_map.wall_marker_dict = self.wall_marker_dict

        # Polling
        self.set_polling_interval(0.025)  # for kine and motion model update

        # Launch viewers
        if self.launch_cam_viewer:
            ensure_viewer(
                self.robot,
                'cam_viewer',
                lambda: CamViewer(self.robot, user_annotate_function=self.user_annotate),
            )

        if self.launch_worldmap_viewer:
            ensure_viewer(
                self.robot,
                'worldmap_viewer',
                lambda: WorldMapViewer(self.robot),
            )

        if self.launch_particle_viewer:
            ensure_viewer(
                self.robot,
                'particle_viewer',
                lambda: ParticleViewer(self.robot, scale=self.particle_viewer_scale),
            )

        if self.launch_path_viewer:
            print('ensure viewer')
            ensure_viewer(
                self.robot,
                'path_viewer',
                lambda: PathViewer(self.robot, self.robot.rrt),
            )

        if self.speech:
            self.robot.speech_listener.enable()
        else:
            self.robot.speech_listener.disable()

        # Set up DoorPass instance needed by Pilot; do it here to avoid circular dependency issues
        pilot.pilot_global_doorpass_node = pilot.DoorPass()

        # Call parent's start() to launch the state machine by invoking the start node.
        super().start()

    def stop(self):
        self.stop_children()
        super().stop()
        self.robot.erouter.clear()

    def stop_children(self):
        for node in self.children.values():
            node.stop()

    def poll(self):
        # Update robot kinematic description
        #self.robot.kine.get_pose()

        # Handle robot being picked up or put down
        if self.robot.is_picked_up():
            if not self.robot.was_picked_up:
                self.robot.robot0.stop_all_movement()
                self.robot.robot0.sound.play(vex.SoundType.HUAH, 50)
                self.robot.particle_filter.delocalize()
                self.robot.was_picked_up = True
                self.picked_up_handler()
        elif self.robot.was_picked_up:
            self.robot.was_picked_up = False
            self.robot.robot0.inertial.calibrate()
            self.robot.robot0.sound.play(vex.SoundType.DOORBELL, 50)
            self.robot.set_pose(0,0,0,0,reset_particles=False)
            self.put_down_handler()

        if not self.robot.was_picked_up:
            self.robot.particle_filter.move()
            self.robot.particle_filter.look_for_new_landmarks()
                
    def robot_picked_up_default(self):
        pass

    def robot_put_down_default(self):
        print('Robot was put down.')

    def user_image(self,image,gray): pass

    def user_annotate(self,image):
        return image

    def _annotated_metadata(self, status: Optional[dict]) -> dict:
        meta = {
            "timestamp": time.time(),
            "frame_count": getattr(self.robot, "frame_count", None),
            "status": status,
            "pose": getattr(self.robot, "pose", None),
            "camera_resolution": getattr(getattr(self.robot, "camera", None), "resolution", None),
            "annotate_sdk": bool(self.annotate_sdk),
            "scale": int(AIVISION_RESOLUTION_SCALE) or 1,
        }
        try:
            meta["aivision"] = (status or {}).get("aivision")
        except Exception:
            meta["aivision"] = None
        return meta

    def _resolve_status(self):
        status = getattr(self.robot, "status", None)
        try:
            if status is not None and "aivision" in status:
                return status
        except TypeError:
            pass
        robot0 = getattr(self.robot, "robot0", None)
        if robot0 is not None:
            try:
                status0 = getattr(robot0, "status", None)
            except Exception:
                status0 = None
            try:
                if status0 is not None and "aivision" in status0:
                    return status0
            except TypeError:
                pass
        return status

    def _emit_annotated_frame(self, image):
        callback = getattr(self, "annotated_image_callback", None)
        if callback is None:
            return
        status = self._resolve_status()
        overlay_status = status
        annotated = apply_overlays(
            image,
            overlay_status,
            int(AIVISION_RESOLUTION_SCALE) or 1,
            getattr(self.robot, "aruco_detector", None),
        )
        maybe = self.user_annotate(annotated)
        if isinstance(maybe, _np.ndarray) and maybe.ndim == 3:
            annotated = maybe
        self.robot.annotated_image = annotated.copy()
        meta = self._annotated_metadata(status)
        callback(annotated, meta)

    def process_image(self,image):
        # Aruco image processing
        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        if self.aruco:
            self.robot.aruco_detector.process_image(gray)
        # Other image processors can run here if the user supplies them.
        self.user_image(image,gray)
        if self.annotated_image_callback is not None:
            self._emit_annotated_frame(image)
        elif self.force_annotation and (not self.robot.cam_viewer or not self.robot.cam_viewer.is_running()):
            status = self._resolve_status()
            overlay_status = status
            annotated = apply_overlays(
                image,
                overlay_status,
                int(AIVISION_RESOLUTION_SCALE) or 1,
                getattr(self.robot, "aruco_detector", None),
            )
            try:
                maybe = self.user_annotate(annotated)
                if isinstance(maybe, _np.ndarray) and maybe.ndim == 3:
                    annotated = maybe
            except Exception:
                pass
            try:
                self.robot.annotated_image = annotated.copy()
            except Exception:
                pass

################

def runfsm(module_name, running_modules=dict()):
    """runfsm('modname') reloads that module and expects it to contain
    a class of the same name. It calls that class's constructor and then
    calls the instance's start() method."""

    global running_fsm
    if running_fsm:
        running_fsm.stop()

    r_py = re.compile('.*\\.py$')
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
    the_module.robot = evbase.robot_for_loading
    # Class's __init__ method will call setup, which can reference the above variables.
    running_fsm = the_class()
    evbase.robot_for_loading.loop.call_soon_threadsafe(running_fsm.start)
    return running_fsm
