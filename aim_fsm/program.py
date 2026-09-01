from math import pi
import math
import time
from typing import Any, Callable, Optional
import re
from importlib import __import__, import_module, reload
import datetime
import os

try:
    from termcolor import cprint
except:
    def cprint(string, color=None):
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
from .worldmap import WorldMap, DominoObj
from .wall_defs import default_wall_marker_dict
from .particle import *
from .utils import Pose
from viewer.particle_viewer import ParticleViewer
from .rrt import RRT
from viewer.path_viewer import PathViewer
from viewer.camera_overlay import apply_overlays
from .camera import AIVISION_RESOLUTION_SCALE
from . import pilot

running_fsm = None

class StateMachineProgram(StateNode):
    def __init__(self,
                 launch_cam_viewer = True,
                 launch_worldmap_viewer = True,
                 force_annotation = False,   # set to True for annotation even without cam_viewer
                 annotate_sdk = True,        # include annotations for SDK's object detections
                 annotated_scale_factor = 1, # set to 1 to avoid cost of resizing images
                 annotated_image_callback: Optional[Callable[[Any, dict], None]] = None,
                 viewer_crosshairs = False,  # set to True to draw viewer crosshairs
                 speech = True,
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
                 domino = True,
                 domino_labeling = False,
                 domino_conf_threshold = 0.35,
                 standing_weights = "standing.pt",
                 fallen_weights = "fallen.pt",
                 standing_label_weights = "standinghalf.pt",
                 fallen_label_weights = "fallenhalf.pt",
                 frame_skip = 2,
                 focal_length = 396.3,
                 perched_cameras = False,
                 rrt = None,
                 domino_face_weights_path = None,
                 ):
        super().__init__()
        self.name = self.__class__.__name__.lower()
        self.parent = None
        self.robot.robot0.set_xy_position(0,0)
        self.robot.robot0.inertial.set_heading(0)

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
        else:
            self.robot.aruco_detector = None

        self.domino = bool(domino)
        self.domino_labeling = bool(domino_labeling)
        self.domino_conf_threshold = float(domino_conf_threshold)
        self.robot.domino_detector = None
        self._last_image = None
        self._normalize_axis_angle = None

        if self.domino:
            try:
                from aim_fsm.domino import DominoWorldDetector, normalize_axis_angle
                self._normalize_axis_angle = normalize_axis_angle
                current_dir = os.path.dirname(os.path.abspath(__file__))
                parent_dir = os.path.dirname(current_dir)
                
                detector = DominoWorldDetector(
                    conf_threshold=self.domino_conf_threshold,
                    standing_weights=os.path.join(parent_dir, standing_weights),
                    fallen_weights=os.path.join(parent_dir, fallen_weights),
                    standing_label_weights=os.path.join(parent_dir, standing_label_weights),
                    fallen_label_weights=os.path.join(parent_dir, fallen_label_weights),
                    frame_skip=frame_skip,
                )
                for attr in ["focal_length", "focal_length_px", "fx", "fy"]:
                    if hasattr(detector, attr):
                        setattr(detector, attr, focal_length)
                self.robot.domino_detector = detector
            except Exception as exc:
                raise ImportError(f"Unable to initialize DominoWorldDetector: {exc}")

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

        self.launch_worldmap_viewer = launch_worldmap_viewer

    def start(self):
        global running_fsm
        running_fsm = self
        if self.particle_filter is None:
            self.particle_filter = SLAMParticleFilter(self.robot,
                                                     num_particles=self.num_particles,
                                                     landmarks=self.landmarks)
        elif isinstance(self.particle_filter, SLAMParticleFilter):
            self.particle_filter.clear_landmarks()
        self.robot.particle_filter = self.particle_filter

        self.robot.was_picked_up = False
        self.robot.holding = None
        self.robot.fetching = None
        self.robot.robot0.led.on(vex.LightType.ALL_LEDS, vex.Color.TRANSPARENT)
        self.robot.clear_actuators()

        self.robot.world_map.wall_marker_dict = self.wall_marker_dict

        self.set_polling_interval(0.025)

        if self.launch_cam_viewer:
            if not self.robot.cam_viewer:
                self.robot.cam_viewer = \
                    CamViewer(self.robot, user_annotate_function=self.user_annotate)
                self.robot.cam_viewer.start()

        if self.launch_worldmap_viewer:
            if not self.robot.worldmap_viewer is True:
                self.robot.worldmap_viewer = WorldMapViewer(self.robot)
                self.robot.worldmap_viewer.start()

        if self.launch_particle_viewer:
            if not self.robot.particle_viewer:
                self.robot.particle_viewer = \
                    ParticleViewer(self.robot, scale=self.particle_viewer_scale)
                self.robot.particle_viewer.start()

        if self.launch_path_viewer:
            if not self.robot.path_viewer:
                self.robot.path_viewer = PathViewer(self.robot, self.robot.rrt)
            self.robot.path_viewer.start()

        if self.speech:
            self.robot.speech_listener.enable()
        else:
            self.robot.speech_listener.disable()

        pilot.pilot_global_doorpass_node = pilot.DoorPass()

        super().start()

    def stop(self):
        self.stop_children()
        super().stop()
        self.robot.erouter.clear()

    def stop_children(self):
        for node in self.children.values():
            node.stop()

    def poll(self):
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

    def user_image(self, image, gray): 
        detector = getattr(self.robot, "domino_detector", None)
        if detector is None or image is None:
            return
        self._last_image = image
        observations = detector.detect(image, frame_id=getattr(self.robot, "frame_count", None))
        self._update_domino_worldmap(observations)

    def _update_domino_worldmap(self, observations):
        """Persist detected dominoes into the world map every frame, the
        same way modelling.py's update_3d_snapshot did on-demand via 'tm'."""
        world_map = getattr(self.robot, "world_map", None)
        normalize_axis_angle = self._normalize_axis_angle
        if not observations or world_map is None or normalize_axis_angle is None:
            return

        for obs in observations:
            try:
                quad = (
                    _np.array(obs.quad, dtype=_np.float32)
                    if (hasattr(obs, "quad") and obs.quad is not None)
                    else None
                )
                cx, cy = obs.center_xy

                bottom_cy = float(_np.max(quad[:, 1])) if quad is not None else cy
                hit, objpos = world_map.project_image_point_to_world(cx, bottom_cy)
                if objpos is None:
                    continue

                x_mm = float(objpos[0][0])
                y_mm = float(objpos[1][0])

                local_yaw = 0.0
                if quad is not None and len(quad) >= 2:
                    sorted_by_y = sorted(quad, key=lambda pt: pt[1], reverse=True)
                    p_base1, p_base2 = sorted_by_y[0], sorted_by_y[1]
                    if p_base1[0] > p_base2[0]:
                        p_base1, p_base2 = p_base2, p_base1

                    hit1, world1 = world_map.project_image_point_to_world(p_base1[0], p_base1[1])
                    hit2, world2 = world_map.project_image_point_to_world(p_base2[0], p_base2[1])

                    if world1 is not None and world2 is not None:
                        dx_world = float(world2[0][0] - world1[0][0])
                        dy_world = float(world2[1][0] - world1[1][0])
                        local_yaw = math.atan2(dy_world, dx_world)

                world_yaw = normalize_axis_angle(self.robot.pose.theta + local_yaw)

                face_label = obs.face_label if getattr(obs, "face_label", None) else "0-0"
                first_half, second_half = 0, 0
                if "-" in face_label:
                    try:
                        parts = face_label.split("-")
                        first_half, second_half = int(parts[0]), int(parts[1])
                    except ValueError:
                        pass

                halves = [
                    {"count": first_half, "local_y": -12.0, "local_x": 0.0},
                    {"count": second_half, "local_y": 12.0, "local_x": 0.0},
                ]

                obj_id = f"domino_{face_label}"

                if obs.is_fallen:
                    z_mm, height_3d, width_3d = 4.0, 8.0, 24.0
                else:
                    z_mm, height_3d, width_3d = 12.0, 24.0, 24.0

                domino_obj = DominoObj(
                    id=obj_id,
                    x=x_mm,
                    y=y_mm,
                    z=z_mm,
                    theta=world_yaw,
                    face_label=face_label,
                    is_fallen=obs.is_fallen,
                )

                setattr(domino_obj, "length", 48.0)
                setattr(domino_obj, "width", width_3d)
                setattr(domino_obj, "height", height_3d)
                setattr(domino_obj, "thickness", 8.0)
                setattr(domino_obj, "is_fallen", obs.is_fallen)
                setattr(domino_obj, "domino_halves", halves)

                world_map.objects[obj_id] = domino_obj
            except Exception as e:
                print(f"[DOMINO WORLDMAP] Error updating {getattr(obs, 'face_label', '?')}: {e}")

    def user_annotate(self, image):
        out = image.copy()
        detector = getattr(self.robot, "domino_detector", None)
        if detector is None:
            return out

        observations = detector.latest_observations()
        overlay = out.copy()

        for idx, obs in enumerate(observations):
            mask_pts = None

            if hasattr(obs, "quad") and obs.quad is not None and len(obs.quad) == 4:
                mask_pts = _np.array(obs.quad, dtype=_np.int32).reshape((-1, 1, 2))

            poly_color = (255, 200, 0) if obs.is_fallen else (0, 230, 110)
            border_color = (255, 255, 0) if obs.is_fallen else (0, 255, 120)

            if mask_pts is not None:
                cv2.fillPoly(overlay, [mask_pts], color=poly_color)
                cv2.polylines(out, [mask_pts], isClosed=True, color=border_color, thickness=2, lineType=cv2.LINE_AA)

            cx, cy = obs.center_xy
            status_str = "FALLEN" if obs.is_fallen else "STANDING"
            
            label = f"Domino.{chr(ord('a') + idx)} ({status_str})"
            if hasattr(obs, "face_label") and obs.face_label:
                label += f" [{obs.face_label}]"
            
            dist_str = f"{obs.distance_cm:.1f}cm" if hasattr(obs, "distance_cm") else ""

            cv2.circle(out, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(out, label, (int(cx) - 50, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2, cv2.LINE_AA)
            if dist_str:
                cv2.putText(out, dist_str, (int(cx) - 20, int(cy) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.addWeighted(overlay, 0.30, out, 0.70, 0, out)
        return out

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

    def process_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        if self.aruco and self.robot.aruco_detector is not None:
            self.robot.aruco_detector.process_image(gray)
        self.user_image(image, gray)
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
                  (module_name, e))
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
                   (module_name, module_name), color="yellow")
    except: pass

    the_module = running_modules[module_name]
    the_class = the_module.__getattribute__(module_name) \
                if module_name in dir(the_module) else None
    if isinstance(the_class, type) and issubclass(the_class, StateNode) and not issubclass(the_class, StateMachineProgram):
        cprint("%s is not an instance of StateMachineProgram.\n" % module_name, color="red")
        return
    if not isinstance(the_class, type) or not issubclass(the_class, StateMachineProgram):
        cprint("Module %s does not contain a StateMachineProgram named %s.\n" %
              (module_name, module_name), color="red")
        return
    the_module.robot = evbase.robot_for_loading
    running_fsm = the_class()
    evbase.robot_for_loading.loop.call_soon_threadsafe(running_fsm.start)
    return running_fsm
