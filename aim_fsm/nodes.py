import time
import asyncio
import inspect
import types
import random
import numpy as np

import cv2

from math import pi, sqrt, atan2, inf, nan
from multiprocessing import Process, Queue

from . import vex
from . import evbase
from .base import *
from .events import *
#from .geometry import wrap_angle
#from .worldmap import WorldObject, FaceObj, CustomMarkerObj

#________________ Ordinary Nodes ________________

class ParentCompletes(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to complete' % (self, self.parent))
        if self.parent:
            self.parent.post_completion()

class ParentSucceeds(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to succeed' % (self, self.parent))
        if self.parent:
            self.parent.post_success()

class ParentFails(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to fail' % (self, self.parent))
        if self.parent:
            self.parent.post_failure()

class Iterate(StateNode):
    """Iterates over an iterable, posting DataEvents.  Completes when done."""
    def __init__(self,iterable=None):
        super().__init__()
        self.iterable = iterable

    class NextEvent(Event): pass

    def start(self,event=None):
        if self.running: return
        super().start(event)
        if isinstance(event, DataEvent):
            self.iterable = event.data
        if isinstance(self.iterable, int):
            self.iterable = range(self.iterable)
        if self.iterable is None:
            raise ValueError('~s has nothing to iterate on.' % repr(self))
        if not isinstance(event, self.NextEvent):
            self.iterator = self.iterable.__iter__()
        try:
            value = next(self.iterator)
        except StopIteration:
            self.post_completion()
            return
        self.post_data(value)

class Print(StateNode):
    "Argument can be a string, or a function to be evaluated at print time."
    def __init__(self,spec=None):
        super().__init__()
        self.spec = spec

    def start(self,event=None):
        super().start(event)
        if isinstance(self.spec, types.FunctionType):
            text = self.spec()
        else:
            text = self.spec
        if text is None and isinstance(event, DataEvent):
            text = repr(event.data)
        print(text)
        self.post_completion()

class SaveImage(StateNode):
    "Save an image to a file."

    def __init__(self, filename="image", filetype="jpg", counter=0, verbose=True):
        super().__init__()
        self.filename = filename
        self.filetype = filetype
        self.counter = counter
        self.verbose = verbose

    def start(self,event=None):
        super().start(event)
        fname = self.filename
        if isinstance(self.counter, int):
            fname = fname + str(self.counter)
            self.counter = self.counter + 1
        fname = fname + "." + self.filetype
        image = np.array(self.robot.world.latest_image.raw_image)
        cv2.imwrite(fname, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        if self.verbose:
            print('Wrote',fname)


#________________ Actions ________________

class ActionNode(StateNode):
    def complete(self,actuator):
        actuator.unlock(self)
        self.post_completion()

class Kick(ActionNode):
    def __init__(self, kicktype=vex.KickType.SOFT):
        super().__init__()
        self.kicktype = kicktype

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['kick'].kick(self, self.kicktype)

class Turn(ActionNode):
    def __init__(self, angle_deg, turn_speed=None):
        super().__init__()
        self.angle_deg = angle_deg
        self.turn_speed = turn_speed

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['drive'].turn(self, self.angle_deg*pi/180, self.turn_speed)


class Forward(ActionNode):
    def __init__(self, distance_mm, drive_speed=None):
        super().__init__()
        self.distance_mm = distance_mm
        self.drive_speed = drive_speed
    
    def start(self, event=None):
        super().start(event)
        self.robot.actuators['drive'].forward(self, self.distance_mm, self.drive_speed)


class Say(ActionNode):
    """Speaks some text, then posts a completion event."""

    class SayDataEvent(Event):
        def __init__(self,text=None):
            self.text = text

    def __init__(self, text="I'm speechless", abort_on_stop=False):
        self.text = text
        super().__init__()

    def start(self,event=None):
        if self.running: return
        if isinstance(event, self.SayDataEvent):
            utterance = event.text
        else:
            utterance = self.text
        if isinstance(utterance, (list,tuple)):
            utterance = random.choice(utterance)
        if not isinstance(utterance, str):
            utterance = repr(utterance)
        self.utterance = utterance
        super().start(event)
        print("Speaking: '",utterance,"'",sep='')

        self.robot.actuators['sound'].say_text(self, self.utterance)


class PlaySound(ActionNode):
    def __init__(self, sound=vex.SoundType.TOLLBOOTH, volume=1):
        self.sound = sound
        self.volume = volume
        super().__init__()

    def start(self,event=None):
        super().start(event)
        self.robot.actuators['sound'].play_sound(self, self.sound, self.volume)

class PlaySoundFile(ActionNode):
    def __init__(self, filepath):
        self.filepath = filepath
        super().__init__()

    def start(self,event=None):
        super().start(event)
        self.robot.actuators['sound'].play_sound_file(self, self.filepath)



class AbortAllActions(StateNode):
    def start(self,event=None):
        super().start(event)
        self.robot.abort_all_actions()
        self.post_completion()



#________________ Multiprocessing ________________

class LaunchProcess(StateNode):

    def __init__(self):
        super().__init__()
        self.process = None

    @staticmethod
    def process_workhorse(reply_token):
        """
        Override this static method with the code to do your computation.
        The method must be static because we can't pickle methods of StateNode
        instances.
        """
        print('*** Failed to override process_workhorse for LaunchProcess node ***')
        print('Sleeping for 2 seconds...')
        time.sleep(2)
        # A process returns its result to the caller as an event.
        result = 42

        LaunchProcess.post_event(reply_token,DataEvent(result))  # source must be None for pickling
        LaunchProcess.post_event(reply_token,CompletionEvent()) # we can post more than one event

    @staticmethod
    def post_event(reply_token,event):
        id,queue = reply_token
        event_pair = (id, event)
        queue.put(event_pair)

    def create_process(self, reply_token):
        p = Process(target=self.__class__.process_workhorse,
                    args=[reply_token])
        return p

    def start(self, event=None):
        super().start(event)
        reply_token = (id(self), self.robot.erouter.interprocess_queue)
        self.process = self.create_process(reply_token)
        self.robot.erouter.add_process_node(self)
        self.process.start()
        print('Launched', self.process)

    def stop(self):
        if self.process:
            print('Exiting',self.process,self.process.is_alive())
            self.process = None
        super().stop()
        self.robot.erouter.delete_process_node(self)

