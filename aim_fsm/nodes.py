import time
import asyncio
import types
import random
import numpy as np
from math import pi, sqrt, atan2, inf, nan
import re

import cv2

from multiprocessing import Process, Queue

import vex
from . import evbase
from .base import *
from .events import *
from .geometry import Distance, Angle, wrap_angle
from .worldmap import WorldObject

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
    def __init__(self,spec=None, prefix=''):
        super().__init__()
        self.spec = spec
        self.prefix = prefix

    def start(self,event=None):
        super().start(event)
        if isinstance(self.spec, types.FunctionType):
            text = self.spec()
        else:
            text = self.spec
        if text is None and isinstance(event, DataEvent):
            text = self.prefix + repr(event.data)
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


class AskGPT(StateNode):
    "Send a query to GPT"

    def __init__(self, query_text=None):
        super().__init__()
        self.query_text = query_text

    def start(self, event=None):
        super().start(event)
        if isinstance(event, SpeechEvent):
            self.query_text = event.string
        self.robot.ask_gpt(self.query_text)


class SendGPTCamera(StateNode):
    "Send current camera image to GPT"

    def __init__(self, instruction=None):
        super().__init__()
        self.instruction = instruction

    def start(self, event=None):
        super().start(event)
        self.robot.send_gpt_camera(instruction=self.instruction)
        self.post_completion()


class AskGPTCamera(StateNode):
    "Send a query to GPT"

    def __init__(self, query_text=None):
        super().__init__()
        self.query_text = query_text

    def start(self, event=None):
        super().start(event)
        if isinstance(event, SpeechEvent):
            self.query_text = event.string
        self.robot.ask_gpt_camera(self.query_text)


class GPTOneShot(StateNode):
    def __init__(self, query_text=None):
        super().__init__()
        self.query_text = query_text
        self.image = None

    def start(self, event=None):
        super().start(event)
        self.robot.gpt_oneshot(self.query_text, self.image)


class TagDetection(StateNode):
    def __init__(self, enabled=True):
        super().__init__()
        self.enabled = enabled

    def start(self, event=None):
        super().start(event)
        self.robot.robot0.vision.tag_detection(self.enabled)
        self.post_completion()


#________________ Actions ________________

# Actions are implemented by the robot's actuators.

class AbortAllActions(StateNode):
    def start(self,event=None):
        super().start(event)
        self.robot.abort_all_actions()
        self.post_completion()


class ActionNode(StateNode):
    def unlock_held_actuators(self):
        for actuator in self.robot.actuators.values():
            actuator.unlock_if_held(self)

    def complete(self):
        self.unlock_held_actuators()
        self.post_completion()

    def stop(self):
        self.unlock_held_actuators()
        super().stop()


class Forward(ActionNode):
    def __init__(self, distance_mm=0.0, drive_speed=None):
        super().__init__()
        self.distance_mm = float(distance_mm)
        self.drive_speed = drive_speed
    
    def start(self, event=None):
        if isinstance(event,DataEvent) and isinstance(event.data, Distance):
            self.distance_mm = event.data.value
        super().start(event)
        self.robot.actuators['drive'].forward(self, self.distance_mm, self.drive_speed)


class Sideways(ActionNode):
    def __init__(self, distance_mm=0.0, drive_speed=None):
        super().__init__()
        self.distance_mm = distance_mm
        self.drive_speed = drive_speed
    
    def start(self, event=None):
        if isinstance(event,DataEvent) and isinstance(event.data, Distance):
            self.distance_mm = event.data.value
        super().start(event)
        self.robot.actuators['drive'].sideways(self, self.distance_mm, self.drive_speed)


class Turn(ActionNode):
    def __init__(self, angle_deg=0.0, turn_speed=None):
        super().__init__()
        self.angle_deg = angle_deg
        self.turn_speed = turn_speed

    def start(self, event=None):
        if isinstance(event,DataEvent) and isinstance(event.data, Angle):
            self.angle_deg = event.data.degrees
        super().start(event)
        self.robot.actuators['drive'].turn(self, self.angle_deg*pi/180, self.turn_speed)


class MoveFor(ActionNode):
    def __init__(self, distance_mm=0.0, angle_deg=0.0, drive_speed=None):
        super().__init__()
        self.distance_mm = distance_mm
        self.angle_deg = angle_deg
        self.drive_speed = drive_speed

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['drive'].move_for(self, self.distance_mm, self.angle_deg, self.drive_speed)


class MoveAt(ActionNode):
    def __init__(self, angle_deg=0, drive_speed=None):
        super().__init__()
        self.angle_deg = angle_deg
        self.drive_speed = drive_speed

    def start(self, event=None):
        super().start(event)
        self.change_motion(self.angle_deg, self.drive_speed)

    def change_motion(self, angle_deg, drive_speed=None):
        self.angle_deg = angle_deg
        self.drive_speed = drive_speed
        if self.running:
            self.robot.actuators['drive'].move_at(self, angle_deg, drive_speed)
        else:
            self.start()

class MoveWithVectors(ActionNode):
    def __init__(self, xvel=0, yvel=0, rvel=0):
        "xvel, yvel, rvel are percentages from -100 to 100)"
        super().__init__()
        self.xvel = xvel
        self.yvel = yvel
        self.rvel = rvel

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['drive'].move_with_vectors(self, self.xvel, self.yvel, self.rvel)

class DriveArc(ActionNode):
    def __init__(self, radius, angle=None, distance=None, speed=1.0):
        super().__init__()
        self.radius = radius
        self.distance = distance
        self.angle = angle
        self.speed = speed

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['drive'].drive_arc(self, self.radius, self.angle,
                                                self.distance, self.speed)
        # for debugging
        self.arc_program = self.robot.actuators['drive'].arc_program


class SpinWheels(ActionNode):
    def __init__(self, left_vel=0, right_vel=0, back_vel=0):
        super().__init__()
        self.left_vel = left_vel
        self.right_vel = right_vel
        self.back_vel = back_vel

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['drive'].spin_wheels(self, self.left_vel, self.right_vel, self.back_vel)


class DrivePath(ActionNode):
    def __init__(self, path = []):
        super().__init__()
        self.path = path

    def start(self,event=None):
        super().start(event)
        if isinstance(event, DataEvent) and isinstance(event.data,(list,tuple)):
            self.path = event.data
        if len(self.path) == 0:
            raise ValueError('Node %s has a null path' % repr(self))
        print('DrivePath: path=', path)
        self.path_index = 0
        self.cur = self.path[self.path_index]
        self.prev = None


class ObjectSpecNode():
    "Supply a get_object_from_spec method for subclasses to use"
    def get_object_from_spec(self,spec):
        if isinstance(spec, WorldObject):
            obj = spec
        elif isinstance(spec,str):
            if spec in self.robot.world_map.objects:  # spec is an object id
                obj = self.robot.world_map.objects[spec]
            else:
                pat = re.compile(spec)
                candidates = [o for o in self.robot.world_map.objects.values() if pat.match(o.name) and o.is_valid]
                obj = None
        elif isinstance(spec,type) and issubclass(spec,WorldObject):
            candidates = [o for o in self.robot.world_map.objects.values() if isinstance(o,spec) and o.is_valid]
            obj = None
        else:
            raise TypeError(f'{self.__class__.__name__} requires an object name spec, object, or object class, not {spec}')
        x = self.robot.pose.x
        y = self.robot.pose.y
        if obj is None and candidates:
            distances = [(o.pose.x - x)**2 + (o.pose.y - y)**2 for o in candidates]
            index = np.argmin(distances)
            obj = candidates[index]
        return obj


class TurnToward(Turn, ObjectSpecNode):
    def __init__(self, object_spec=None):
        super().__init__()
        self.object_spec = object_spec

    def start(self, event=None):
        if isinstance(event, DataEvent):
            spec = event.data
        else:
            spec = self.object_spec
        if spec is None:
            self.angle_deg = 0
            super().start(event)
            self.post_failure()
            return
        obj = self.get_object_from_spec(spec)
        if obj is None:
            self.angle_deg = 0
            super().start(event)
            self.post_failure()
            return
        dx = obj.pose.x - self.robot.pose.x
        dy = obj.pose.y - self.robot.pose.y
        angle = wrap_angle(atan2(dy,dx) - self.robot.pose.theta)
        self.angle_deg = angle*180/pi
        super().start(event)


class SoftKick(ActionNode):
    def __init__(self):
        super().__init__()

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['kick'].kick(self, vex.KickType.SOFT)

class MediumKick(ActionNode):
    def __init__(self):
        super().__init__()

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['kick'].kick(self, vex.KickType.MEDIUM)


class HardKick(ActionNode):
    def __init__(self):
        super().__init__()

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['kick'].kick(self, vex.KickType.HARD)


class PlaceKick(ActionNode):
    def __init__(self):
        super().__init__()

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['kick'].place(self)


class Kick(ActionNode):
    def __init__(self, kicktype=vex.KickType.MEDIUM):
        super().__init__()
        self.kicktype = kicktype

    def start(self, event=None):
        super().start(event)
        self.robot.actuators['kick'].kick(self, self.kicktype)


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
        print(f"Speaking: '{utterance}'")
        if utterance.strip() == '':
            self.post_completion()
            return
        self.robot.actuators['sound'].say_text(self, self.utterance)


class PlaySound(ActionNode):
    def __init__(self, sound=vex.SoundType.DOORBELL):
        self.sound = sound
        super().__init__()

    def start(self,event=None):
        super().start(event)
        self.robot.actuators['sound'].play_sound(self, self.sound)


class PlaySoundFile(ActionNode):
    def __init__(self, filepath):
        self.filepath = filepath
        super().__init__()

    def start(self,event=None):
        super().start(event)
        self.robot.actuators['sound'].play_sound_file(self, self.filepath)


class PlayNotes(ActionNode):
    DURATION = 200 # msecs

    convert_flat = {'Ab' : 'G#',
                    'Bb' : 'A#',
                    'Cb' : 'B',
                    'Db' : 'C#',
                    'Eb' : 'D#',
                    'Fb' : 'E',
                    'Gb' : 'F#'}

    def __init__(self, score='C5'):
        super().__init__()
        self.score = score

    def start(self, event=None):
        super().start(event)
        if isinstance(event, DataEvent) and isinstance(event.data, (str,list,tuple)):
            score = event.data
        else:
            score = self.score
        if isinstance(score, str):
            self.note_list = [s for s in score.strip().split(' ') if len(s) > 0]
        else:
            self.note_list = score
        self.index = -1
        self.complete()

    def complete(self):
        self.index += 1
        if self.index >= len(self.note_list):
            super().complete()
        else:
            note = self.note_list[self.index]
            # convert flats to sharps
            if len(note) >= 3 and note[1] == 'b':
                if note[0] == 'C':
                    octave = note[2] if note[2] in '5678' else '1'
                    note = 'B' + str(int(octave)-1) + note[3:]
                else:
                    note = self.convert_flat[note[0:2]] + note[2:]
            # handle note duration
            if note[-2:] == '__':
                pitch = note[0:-2]
                duration = self.DURATION * 4
            elif note[-1:] == '_':
                pitch = note[0:-1]
                duration = self.DURATION * 2
            elif note[-1:] == '-':
                pitch = note[0:-1]
                duration = self.DURATION // 2
            else:
                pitch = note
                duration = self.DURATION
            # validate note and play if valid, else complain
            if pitch[0] in 'ABCDEFG' and \
               ( len(pitch) == 2 and pitch[1] in '5678' ) or \
               ( len(pitch) == 3 and pitch[1] == '#' and pitch[2] in '5678'):
                self.robot.actuators['sound'].play_note(self, pitch, duration)
            else:
                print(f"Invalid pitch '{pitch}' in {self}")
                self.complete()


class Glow(ActionNode):
    """Glow(light_type, color) or Glow(light_type, r, g, b)"""
    def __init__(self, *args):
        self.args = args
        super().__init__()

    def start(self, event=None):
        super().start(event)
        try:
            self.robot.actuators['leds'].set_light_color(self, *self.args)
        except Exception as e:
            self.robot.actuators['leds'].unlock(self)
            raise
        self.complete()


class Flash(ActionNode):
    """
    Flash(led_program, duration=None, num_cycles=None)
    where led_program can be any of:
       a color -- sets all LEDs to that color
       a list of steps of form (pattern, duration)
    A pattern can be:
       a color -- applied to all LEDs
       a list of six colors for the six LEDs
    A color can be:
       a vex.Color or vex.DefinedColor
       a triple of (r, g, b) values
    """
    def valid_color(self, color_spec):
        if isinstance(color_spec, (vex.Color,vex.Color.DefinedColor)): return True
        if not isinstance(color_spec, (list,tuple)): return False
        if len(color_spec) != 3: return False
        for c in color_spec:
            if not (isinstance(c, int) and c >= 0): return False
        return True

    def valid_pattern(self, pat):
        if self.valid_color(pat):
            return True
        if not (isinstance(pat, (list,tuple)) and len(pat) == 6):
            return False
        for color_spec in pat:
            if not self.valid_color(color_spec):
                return False
        return True

    def valid_program_step(self, step):
        return isinstance(step,(list,tuple)) and \
            len(step) == 2 and \
            self.valid_pattern(step[0]) and \
            isinstance(step[1], (int,float))
    
    def valid_program(self, prog):
        for step in prog:
            if not self.valid_program_step(step):
                return False
        return True

    def __init__(self, led_program=list(), num_cycles=None, duration=None):
        super().__init__()
        if self.valid_color(led_program):
            led_program = [(led_program, 2)]
        if not self.valid_program(led_program):
            raise ValueError(led_program)
        self.led_program = led_program
        self.duration = duration
        self.num_cycles = num_cycles
        program_duration = sum(step[1] for step in led_program)
        if num_cycles is not None and duration is not None:
            raise ValueError('Cannot specify both num_cycles and duration')
        if num_cycles:
            self.total_duration = num_cycles * program_duration
        elif duration:
            self.total_duration = duration
        else:
            self.total_duration = np.inf

    def start(self,event=None):
        program_duration = sum(step[1] for step in self.led_program)
        if self.num_cycles:
            self.total_duration = self.num_cycles * program_duration
        elif self.duration:
            self.total_duration = self.duration
        else:
            self.total_duration = np.inf
        self.current_step = -1
        self.time_remaining = self.total_duration
        super().start(event)
        if len(self.led_program) == 0:
            self.complete()
            return
        self.poll()

    def complete(self):
        self.robot.actuators['leds'].set_light_color(self, vex.LightType.ALL_LEDS, vex.Color.TRANSPARENT)
        super().complete()

    def poll(self):
        if self.time_remaining <= 0:
            self.set_polling_interval(None)
            self.complete()
            return
        self.current_step = (1 + self.current_step) % len(self.led_program)
        (step_pattern, step_dur) = self.led_program[self.current_step]
        if self.time_remaining < step_dur:
            step_dur = self.time_remaining
        self.set_polling_interval(step_dur)
        leds_actuator = self.robot.actuators['leds']
        if isinstance(step_pattern, (vex.Color,vex.Color.DefinedColor)):
            leds_actuator.set_light_color(self, vex.LightType.ALL_LEDS, step_pattern)
        elif isinstance(step_pattern, (tuple,list)) and len(step_pattern) == 3:
            leds_actuator.set_light_color(self, vex.LightType.ALL_LEDS, *step_pattern)
        else:
            lights = (vex.LightType.LED1, vex.LightType.LED2,
                      vex.LightType.LED3, vex.LightType.LED4,
                      vex.LightType.LED5, vex.LightType.LED6)
            for (light, color) in zip(lights, step_pattern):
                leds_actuator.set_light_color(self, light, color)
        self.time_remaining -= step_dur


class ShowEmoji(ActionNode):
    def __init__(self, emoji=vex.EmojiType.HAPPY, direction=vex.EmojiLookType.LOOK_FORWARD):
        super().__init__()
        self.emoji = emoji
        self.direction = direction
        
    def start(self, event=None):
        super().start(event)
        self.robot.actuators['display'].show_emoji(self, self.emoji, self.direction)
        self.complete()


class HideEmoji(ActionNode):
    def start(self, event=None):
        super().start(event)
        self.robot.actuators['display'].hide_emoji(self)
        self.complete()


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

