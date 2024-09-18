import asyncio

from gtts import gTTS

class Actuator():
    class ActuatorLocked(Exception): pass
    class ActuatorNotHeld(Exception): pass

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
        elif self.holder is node:
            return True
        else:
            raise self.ActuatorLocked(self)

    def unlock(self, node):
        if self.holder is node:
            self.holder = None
        else:
            raise self.ActuatorNotHeld()

    def status_update(self): pass

    def complete(self):
        print(f"{self.name} completes, holder {self.holder}")
        if self.holder:
            self.holder.complete(self)

class DriveActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'drive')

    def stop(self):
        self.robot.robot0.stop_drive()

    def status_update(self):
        if self.holder and not self.robot.robot0.is_moving():
            self.holder.complete(self)
            self.holder = None

    def turn(self, node, angle_rads, turn_speed=None):
        self.lock(node)
        self.robot.turn(angle_rads, turn_speed=turn_speed)

    def forward(self, node, distance_mm, drive_speed=None):
        self.lock(node)
        self.robot.forward(distance_mm, drive_speed=drive_speed)


class SoundActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'sound')
        self.playing = False

    def status_update(self):
        if self.robot.robot0.is_sound_active():
            self.playing = True
        else:
            if self.playing is True:
                self.playing = False
                self.complete()

    def say_text(self, node, text):
        self.lock(node)
        self.robot.loop.call_soon_threadsafe(self.launch_text_to_mp3, text)

    def launch_text_to_mp3(self, text):
        self.robot.loop.create_task(self.text_to_mp3(text))

    async def text_to_mp3(self, text):
        filepath = "/tmp/vex_speech.mp3"
        tts = gTTS(text=text, lang='en')
        tts.save(filepath)
        self.robot.robot0.play_sound_file(filepath)

    def play_sound(self, node, sound, volume=1):
        self.lock(node)
        self.robot.robot0.play_sound(sound, volume)

    def play_sound_file(self, node, filepath):
        self.lock(node)
        self.robot.robot0.play_sound_file(filepath)


class KickActuator(Actuator):
    KICK_DURATION = 0.25 # seconds

    def __init__(self, robot):
        super().__init__(robot, 'kick')

    def kick(self, node, kicktype):
        self.lock(node)
        self.robot.robot0.kick(kicktype)
        self.robot.loop.call_soon_threadsafe(self.set_delayed_completion)

    def set_delayed_completion(self):
        self.robot.loop.create_task(self.delayed_completion())

    async def delayed_completion(self):
        await asyncio.sleep(self.KICK_DURATION)
        if self.holder:
            self.holder.complete(self)


class LEDsActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'leds')

    def stop(self):
        self.robot.robot0.clear_leds()

