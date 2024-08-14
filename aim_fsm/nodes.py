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

# Actions

class Kick(StateNode):
    KICK_DURATION = 0.5 # seconds

    def start(self,event=None):
        super().start(event)
        self.robot.actuators['kick'].lock(self)
        self.robot.robot0.kick(vex.KickType.SOFT)
        self.robot.loop.call_soon_threadsafe(self.delayed_completion)

    def delayed_completion(self):
        self.robot.loop.create_task(self.delayed_completion2())

    async def delayed_completion2(self):
        await asyncio.sleep(self.KICK_DURATION)
        self.robot.actuators['kick'].unlock()
        self.post_completion()

class AbortAllActions(StateNode):
    def start(self,event=None):
        super().start(event)
        self.robot.abort_all_actions()
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



class Say(StateNode):
    """Speaks some text, then posts a completion event."""

    class SayDataEvent(Event):
        def __init__(self,text=None):
            self.text = text

    def __init__(self, text="I'm speechless",
                 abort_on_stop=False, **action_kwargs):
        self.text = text
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop)

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
        print("Speaking: '",utterance,"'",sep='')
        super().start(event)

    def action_launcher(self):
        if self.utterance.rstrip() == '':
            # robot.say_text() action would fail on empty string
            self.post_completion()
            return None
        else:
            return self.robot.say_text(self.utterance, **self.action_kwargs)


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

