import threading
import numpy as np
import cv2

from . import aim
from . import vex
from .camera import *
from .aim_kin import *
from .evbase import EventRouter
from .events import *
from .actuators import *
from .aruco import *
from .worldmap import *
from . import program

class Robot():
    def __init__(self, robot0=None, loop=None):
        if robot0 is None:
            robot0 = aim.Robot()
        robot0.inertial.calibrate()
        robot0.set_pose(0,0,0)
        self.robot0 = robot0
        self.loop = loop
        self.camera = Camera()
        self.kine = AIMKinematics(self)
        self.world_map = WorldMap(self)
        self.aruco = None
        self.status = self.robot0._ws_status_thread.current_status['robot']
        acts = [DriveActuator(self), SoundActuator(self), KickActuator(self), LEDsActuator(self)]
        self.actuators = {act.name : act for act in acts}
        self.erouter = EventRouter(self)
        self.cam_viewer = None
        self.touch = '0x00'
        robot0._ws_status_thread.callback = self.status_callback
        self.camera_image = None
        robot0._ws_img_thread.callback = self.image_callback
        robot0.get_camera_image()  # start the image stream

    def status_callback(self):
        #print(f"status_callback in {threading.current_thread().native_id}")
        self.loop.call_soon_threadsafe(self.status_update)

    def status_update(self):
        self.old_status = self.status
        self.status = self.robot0._ws_status_thread.current_status['robot']
        """
        gyro = sum([abs(float(self.status['gyro_rate'][axis])) for axis in 'xyz'])
        accel = abs(float(self.status['pitch'])) + abs(float(self.status['roll']))
        if accel > 10:
            self.robot0.stop_drive()
            self.robot0.play_sound(vex.SoundType.ALARM, 100)
        """
        self.x = float(self.status['robot_y'])
        self.y = -float(self.status['robot_x'])
        self.z = 0
        heading = 360 - float(self.status['heading'])
        if heading == 360:
            print(f"*** {self.status}")
        if heading > 180:
            heading = heading - 360
        self.theta = heading / 180 * pi
        self.update_actuators()
        self.world_map.update()
        t = self.status['touch_flags']
        if self.touch != t:
            print(f"status_update in {threading.current_thread().native_id}")
            print(t)
            self.touch = t
            touch_event = TouchEvent(self.status['touch_x'],
                                     self.status['touch_y'],
                                     self.status['touch_flags'])
            self.erouter.post(touch_event)

    def update_actuators(self):
        for act in self.actuators.values():
            act.status_update()

    def image_callback(self):
        ws = self.robot0._ws_img_thread
        image_bytes = ws.image_list[ws._next_image_index]
        image_array = np.frombuffer(image_bytes, dtype='uint8')
        self.camera_image = cv2.imdecode(image_array, cv2.IMREAD_UNCHANGED)
        if program.running_fsm:
            program.running_fsm.process_image(self.camera_image)

    def turn(self, angle_rads, turn_speed=None):
        if angle_rads > 0:
            turntype = vex.TurnType.LEFT
        else:
            turntype = vex.TurnType.RIGHT
        print(f"turn for {angle_rads} radians = {angle_rads*180/pi} deg")
        print(f"turning in thead {threading.current_thread().native_id}")
        self.robot0.turn_for(turntype, abs(angle_rads)*180/pi, turn_speed=turn_speed, wait=False)

    def forward(self, distance_mm, drive_speed=None):
        angle_zero = 0
        self.robot0.drive_for(distance_mm, angle_zero, drive_speed=drive_speed, wait=False)
