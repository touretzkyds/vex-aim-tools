import threading
import numpy as np
import cv2

from . import aim
from .evbase import EventRouter
from .events import *

class Actuator():
    def __init__(self, robot, name, stop_fn = lambda : None):
        self.robot = robot
        self.name = name
        self.holder = None
        self.stop_fn = stop_fn

    def __repr__(self):
        return f"<Actuator {self.name}>"

    def lock(self, node):
        if self.holder is None:
            self.holder = node
            return True
        else:
            return False

    def complete(self):
        print(f"{self.name} completes, holder {self.holder}")
        if self.holder:
            self.holder.complete()
        self.holder = None

class DriveActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'drive')

    def stop(self):
        self.robot.robot0.stop_drive()

class SoundActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'sound')

class KickActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'kick')

class LEDsActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'leds')

    def stop(self):
        self.robot.robot0.clear_leds()

#================================================================

class Robot():
    def __init__(self, robot0=None, loop=None):
        if robot0 is None:
            robot0 = aim.Robot()
        self.robot0 = robot0
        self.loop = loop
        acts = [DriveActuator(self), SoundActuator(self), KickActuator(self), LEDsActuator(self)]
        self.actuators = {act.name : act for act in acts}
        self.erouter = EventRouter()
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
        self.status = self.robot0._ws_status_thread.current_status['robot']
        t = self.status['touch_flags']
        if self.touch != t:
            print(f"status_update in {threading.current_thread().native_id}")
            print(t)
            self.touch = t
            touch_event = TouchEvent(self.status['touch_x'],
                                     self.status['touch_y'],
                                     self.status['touch_flags'])
            self.erouter.post(touch_event)
        if not self.robot0.is_driving():
            drive_act = self.actuators['drive']
            if drive_act.holder is not None:
                drive_act.complete()

    def image_callback(self):
        ws = self.robot0._ws_img_thread
        image_bytes = ws.image_list[ws._next_image_index]
        image_array = np.frombuffer(image_bytes, dtype='uint8')
        self.camera_image = cv2.imdecode(image_array, cv2.IMREAD_UNCHANGED)
