import sys
import threading
import signal
import numpy as np
import cv2

import vex
#from . import aim

from .camera import *
from .aim_kin import *
from .evbase import EventRouter
from .events import *
from .actuators import *
from .speech_rec import SpeechListener
from .thesaurus import Thesaurus
from .openai_client import OpenAIClient
from .aruco import *
from .particle import SLAMParticleFilter
from .worldmap import *
from .rrt import RRT
from .path_planner import PathPlanner
from .utils import Pose, PoseEstimate
from .geometry import wrap_angle_deg
from . import program
from .pilot import DoorPass

class Robot():
    def __init__(self, robot0=None, loop=None, host="192.168.4.1",
                 launch_speech_listener=True):
        if robot0 is None:
            robot0 = vex.Robot(host=host)
        signal.signal(signal.SIGINT, self.signal_handler)
        self.robot0 = robot0
        self.robot0.inertial.calibrate()
        self.robot0.inertial.set_heading(0)
        self.robot0.set_xy_position(0,0)
        self.sound_volume = 100
        self.pose = Pose(0,0,0,0)
        self.loop = loop
        self.holding = None   # object being held
        self.last_held_time = 0
        self.camera = Camera()
        self.kine = AIMKinematics(self)
        self.world_map = WorldMap(self)
        self.worldmap_viewer = None
        self.particle_filter = SLAMParticleFilter(self)
        self.particle_viewer = None
        self.rrt = RRT(self)
        self.path_planner= PathPlanner(self)
        self.path_viewer = None
        self.aruco_detector = None
        acts = [DriveActuator(self), SoundActuator(self), KickActuator(self),
                LEDsActuator(self), DisplayActuator(self)]
        self.actuators = {act.name : act for act in acts}
        self.erouter = EventRouter(self)
        self.cam_viewer = None
        self.touch = '0x00'
        self.flask_thread = None
        self.openai_client = OpenAIClient(self)
        self.camera_image = None
        self.frame_count = 0    # camera images received so far
        self.moving_frame = 0   # last camera image when robot was moving
        self.status = self.robot0._ws_status_thread.current_status['robot']
        robot0._ws_status_thread.callback = self.status_callback
        robot0._ws_img_thread.callback = self.image_callback
        robot0.vision.get_camera_image()  # start the image stream
        robot0.vision.tag_detection(False)  # tag detection greatly slows the image stream
        self.thesaurus = Thesaurus()
        self.speech_listener = SpeechListener(self, self.thesaurus, debug=False)
        if launch_speech_listener:
            self.loop.call_soon_threadsafe(self.speech_listener.start)

    def signal_handler(self, x,y):
        self.abort_all_actions()
        if program.running_fsm:
            print('SIGINT: stopping', program.running_fsm.name)
            program.running_fsm.stop()
        else:
            print('--->>> Signal SIGINT')

    def status_callback(self):
        self.loop.call_soon_threadsafe(self.status_update)

    def status_update(self):
        self.old_status = self.status
        self.status = self.robot0._ws_status_thread.current_status['robot']
        """
        gyro = sum([abs(float(self.status['gyro_rate'][axis])) for axis in 'xyz'])
        accel = abs(float(self.status['pitch'])) + abs(float(self.status['roll']))
        if accel > 10:
            self.robot0.stop_all_motion()
            self.robot0.play_sound(vex.SoundType.ALARM, 100)
        """
        heading = 360 - self.robot0.inertial.get_heading()
        if heading > 180:
            heading = heading - 360
        theta = heading / 180 * pi
        # self.pose = PoseEstimate(self.robot0.get_y(), -self.robot0.get_x_position(), 0, theta)

        self.battery_percentage = self.status['battery']
        self.update_actuators()
        if not self.robot0.is_stopped():
            self.moving_frame = self.frame_count
            self.world_map.pause_visibility()
        else:
            if self.frame_count > self.moving_frame + 1:
                self.world_map.update()
                # turn on visibility AFTER we've processed a camera frame
                self.world_map.pause_visibility(False)
        t = self.status['touch_flags']
        if self.touch != t:
            #print(f"status_update in {threading.current_thread().native_id}")
            #print(t)
            self.touch = t
            touch_event = TouchEvent(self.status['touch_x'],
                                     self.status['touch_y'],
                                     self.status['touch_flags'])
            self.erouter.post(touch_event)

    def set_sound_volume(self, volume):
        if isinstance(volume,int) and volume >= 0 and volume <= 100:
            self.sound_volume = volume
        else:
            raise ValueError(f'Volume must be an integer from 0 to 100, not \'{volume}\'')

    def set_pose(self, x, y, z, theta, reset_particles=True):
        self.pose = PoseEstimate(x, y, z, theta)
        x0, y0, heading0 = -y, x, (360 - theta * 180/pi)  # convert to VEX frame
        self.robot0.set_xy_position(x0,y0)
        self.robot0.inertial.set_heading(heading0)
        if self.particle_filter and reset_particles:
            self.particle_filter.set_pose(x,y,theta)

    def abort_all_actions(self):
        self.robot0.stop_all_movement()
        self.clear_actuators()

    def update_actuators(self):
        for act in self.actuators.values():
            act.status_update()

    def clear_actuators(self):
        for act in self.actuators.values():
            act.clear()

    def image_callback(self):
        ws = self.robot0._ws_img_thread
        image_bytes = ws.image_list[ws._next_image_index]
        image_array = np.frombuffer(image_bytes, dtype='uint8')
        self.camera_image = cv2.cvtColor(cv2.imdecode(image_array, cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB)
        self.frame_count += 1
        if program.running_fsm:
            program.running_fsm.process_image(self.camera_image)

    def restart_img_thread(self):
        self.loop.call_soon_threadsafe(self.robot0._ws_img_thread.stop_stream)
        self.loop.call_later(0.5, self.loop.call_soon_threadsafe, self.robot0._ws_img_thread.start_stream)
        print(self.robot0._ws_img_thread)

    def is_picked_up(self):
        """
        This function could be smarter about deciding when the robot has been
        put down.  The attitude_threshold should be raised to 7 degrees, but
        the robot should be required to hold its attitude to within 1 degree for
        at least 1 second.  This will handle cases where the robot is put down
        partially on top of something like a pen, so it's a little tilted.
        """
        gyro = self.status['gyro_rate']
        x = float(gyro['x'])
        y = float(gyro['y'])
        gyro_threshold = 40
        pitch = self.robot0.inertial.get_pitch()
        roll = self.robot0.inertial.get_roll()
        attitude_threshold = 8  # degrees
        if abs(x) > gyro_threshold or abs(y) > gyro_threshold or \
           abs(pitch) > attitude_threshold or abs(roll) > attitude_threshold:
            if not self.was_picked_up:
                print(f"=== [Gyro  x: {x}  y: {y}  threshold: {gyro_threshold}]   " +
                      f"[Inertial  pitch: {pitch}  roll: {roll}  threshold: {attitude_threshold}]")
            return True
        else:
            if self.was_picked_up:
                pass # print(f"*** Gyro  x:{x}  y:{y}  pitch:{pitch}  roll:{roll}")
            return False

    def is_moving(self):
        return not self.robot0.is_stopped()

    def ask_gpt(self, query_text):
        self.openai_client.query(query_text)

    def gpt_note_for_later(self,text):
        self.openai_client.note_for_later(text)

    def send_gpt_camera(self, instruction=None):
        self.openai_client.send_camera_image(instruction=instruction)

    def ask_gpt_camera(self, query_text):
        self.openai_client.camera_query(query_text)

    def gpt_oneshot(self, query_text, image=None):
        self.openai_client.oneshot_query(query_text, image)

    def show_pose(self):
        print(f'Odometry:  {self.robot0.get_y_position():.1f}, ' +
              f'{-self.robot0.get_x_position():.1f} ' +
              f'heading {wrap_angle_deg(-self.robot0.inertial.get_heading()):.1f} deg.', end='')
        print(f'   [ Roll: {self.robot0.inertial.get_roll():.1f}  ' +
              f'Pitch: {self.robot0.inertial.get_pitch():.1f}  ' +
              f'Yaw: {self.robot0.inertial.get_yaw():.1f} ]')
        pf_pose = self.particle_filter.update_pose_estimate()
        var = self.particle_filter.update_pose_variance()
        print(f'Particles: {pf_pose.x:.1f}, ' +
              f'{pf_pose.y:.1f} ' +
              f'heading {pf_pose.theta*180/pi:.1f} deg.  ' +
              f'[{self.particle_filter.state}]   ' +
              f'variance: <{var[0][0,0]:.1f}, {var[0][1,1]:.1f}> ! <{var[1]:.4f}>')
        print()

    def print_raw_odometry(self):
        print(f'x={self.robot0.get_x_position()} y={self.robot0.get_y_position()} hdg={self.robot0.inertial.get_heading()}')
