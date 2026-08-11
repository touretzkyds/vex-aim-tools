import asyncio
import os
import math
from math import pi, sin, cos, atan2

from gtts import gTTS
import google.cloud
from google.cloud import texttospeech

import vex
from .geometry import wrap_angle

class Actuator():
    class ActuatorLocked(Exception): pass
    class ActuatorNotHeld(Exception): pass

    def __init__(self, robot, name, stop_fn = lambda : None):
        self.robot = robot
        self.name = name
        self.holder = None
        self.started = False
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
            raise self.ActuatorLocked(f'{self} locked by {self.holder}')

    def unlock(self, node):
        if self.holder is node:
            self.holder = None
        else:
            raise self.ActuatorNotHeld()

    def unlock_if_held(self, node):
        "Needed if an external event shuts down a node that might have locked the actuator."
        if self.holder is node:
            self.holder = None

    def clear(self):
        self.holder = None
        self.started = False

    def status_update(self): pass

    def complete(self):
        if self.holder:
            self.holder.complete()

class DriveActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'drive')
        self.arc_program = None

    def stop(self):
        self.robot.robot0.stop_all_movement()
        self.arc_program = None

    def status_update(self):
        # Bad timing can cause a just-started motion node to appear to
        # have completed because the robot isn't moving yet; we must
        # wait until robot is seen to be moving before considering
        # looking for a stopped-moving status to detect completion.
        if not self.robot.robot0.is_stopped():
            if not self.started:
                #print('drive actuator: robot started moving for', self.holder)
                self.started = True  # started moving, now wait for completion
            elif self.arc_program:
                current_x = self.robot.robot0.get_y_position()
                current_y = - self.robot.robot0.get_x_position()
                if self.arc_program.is_done(current_x, current_y):
                    self.stop()   # will cause completion once robot is stopped
        elif self.holder and self.started:  # robot has just stopped; signal completion
            #print('drive actuator signaling completion to', self.holder)
            self.holder.complete()
            self.holder = None
            self.started = False
            self.arc_program = None

    def turn(self, node, angle_rads, turn_speed=None):
        self.lock(node)
        self.started = False
        if angle_rads > 0:
            turntype = vex.TurnType.LEFT
        else:
            turntype = vex.TurnType.RIGHT
        self.robot.world_map.pause_visibility()
        #print(f'actuator turn_for({turntype}, {abs(angle_rads)*180/pi}, {turn_speed}, {vex.TurnVelocityUnits.DPS}, {False}) for {self.holder}')
        self.robot.robot0.turn_for(turntype, abs(angle_rads)*180/pi,
                                   turn_speed, vex.TurnVelocityUnits.DPS, False)

    def forward(self, node, distance_mm, drive_speed=None):
        self.lock(node)
        self.started = False
        angle_forward = 0
        self.robot.world_map.pause_visibility()
        self.robot.robot0.move_for(distance_mm, angle_forward,
                                   drive_speed, vex.DriveVelocityUnits.MMPS, False)

    def sideways(self, node, distance_mm, drive_speed=None):
        self.lock(node)
        self.started = False
        angle_leftward = -90
        self.robot.world_map.pause_visibility()
        self.robot.robot0.move_for(distance_mm, angle_leftward,
                                   drive_speed, vex.DriveVelocityUnits.MMPS, False)

    def move_for(self, node, distance_mm, angle_deg, drive_speed=None):
        self.lock(node)
        self.started = False
        self.robot.world_map.pause_visibility()
        self.robot.robot0.move_for(distance_mm, -angle_deg,
                                   drive_speed, vex.DriveVelocityUnits.MMPS, False)

    def move_at(self, node, angle_deg, drive_speed=None):
        self.lock(node)
        self.started = False
        self.robot.world_map.pause_visibility()
        self.robot.robot0.move_at(-angle_deg, drive_speed, vex.DriveVelocityUnits.MMPS)


    def move_with_vectors(self, node, xvel, yvel, rvel):
        self.lock(node)
        self.started = False
        self.robot.world_map.pause_visibility()
        self.robot.robot0.move_with_vectors(xvel, -yvel, -rvel)

    def spin_wheels(self, node, left_vel, right_vel, back_vel):
        print('*** spin_wheels is deprecated and is going away ***')
        self.lock(node)
        self.started = False
        self.robot.world_map.pause_visibility()
        self.robot.robot0.spin_wheels(left_vel, right_vel, back_vel)

    class ArcProgram():
        "Calculations for driving along an arc; used in drive_arc and status_update."
        def __init__(self, start_x, start_y, start_theta, radius, angle, distance, omega):
            self.start_x = start_x
            self.start_y = start_y
            self.start_theta = start_theta
            self.radius = radius
            self.angle = angle
            self.distance = distance
            self.omega = omega
            self.sign_omega = math.copysign(1.0, omega)

            if distance is not None:
                self.target = abs(distance) / radius   # convert to radians
            else:
                self.target = abs(angle)

            self.center_x = start_x + radius * cos(start_theta + pi/2)
            self.center_y = start_y + radius * sin(start_theta + pi/2)

            self.prev_x = start_x
            self.prev_y = start_y
            self.accumulated = 0.0
            
        def is_done(self, current_x, current_y):
            ix = self.prev_x - self.center_x
            iy = self.prev_y - self. center_y
            cx = current_x - self.center_x
            cy = current_y - self.center_y

            # atan2(cross, dot) is robust to the robot drifting off the ideal circle.
            self.accumulated += atan2(ix * cy - iy * cx, ix * cx + iy * cy)
            self.prev_x, self.prev_y = current_x, current_y

            return self.accumulated * self.sign_omega >= self.target

        def __repr__(self):
            return f'<ArcProgram ' + \
                f'start={self.start_x:.1f},{self.start_y:.1f}  ' + \
                f'theta={self.start_theta*180/pi:.1f} deg.  ' + \
                f'center={self.center_x:.1f},{self.center_y:.1f}  ' + \
                f'radius={self.radius:.1f} ' + \
                ('' if self.angle is None else f' angle={self.angle:.3f}') + \
                ('' if self.distance is None else f' distance={self.distance:.1f}') + \
                f'  target={self.target:.3f}>'


    sin60 = math.sqrt(3) / 2

    def drive_arc(self, node, radius, angle=None, distance=None, speed=1.0):
        """
        Set wheel velocities to drive the robot along a circular arc.

        radius:   Turning radius in mm. Positive = arc center to the left (CCW turn).
        angle:    Intended arc angle in radians. Mutually exclusive with distance.
        distance: Intended arc length in mm. Mutually exclusive with angle.
        speed:    Angular rate in rad/s. Default 1.0.

        Negative speed, angle, or distance each reverse the direction of travel;
        two negatives cancel. Termination is the caller's responsibility.
        """

        if angle is not None and distance is not None:
            raise ValueError("Specify angle or distance, not both.")
        if radius == 0:
            raise ValueError("radius must be nonzero.")
        if speed == 0:
            raise ValueError("speed must be nonzero.")

        if angle is not None:
            sign_term = math.copysign(1.0, angle * radius)
        elif distance is not None:
            sign_term = math.copysign(1.0, distance * radius)
        else:
            raise ValueError('Must specify either angle or distance to travel.')

        omega = speed * sign_term
        vx    = omega * radius

        self.lock(node)
        self.started = False
        self.robot.world_map.pause_visibility()

        start_x = self.robot.robot0.get_y_position()
        start_y = - self.robot.robot0.get_x_position()
        start_theta = wrap_angle(-self.robot.robot0.inertial.get_heading()/180 * pi)
        self.arc_program = self.ArcProgram(start_x, start_y, start_theta, radius,
                                           angle, distance, omega)

        wheel_distance = self.robot.kine.wheel_distance
        # v_wheel = -vx*sin(phi) + omega*r  (vy=0 for a pure arc)
        v_lf =  vx * self.sin60 - omega * wheel_distance   # phi = +60 deg
        v_rf = -vx * self.sin60 - omega * wheel_distance   # phi = -60 deg
        v_b  =                  - omega * wheel_distance   # phi = 180 deg, sin(180)=0

        self.robot.robot0.spin_wheels(v_lf, v_rf, v_b)


class SoundActuator(Actuator):
    # ---- Text-to-speech selection ------------------------------------------
    # Edit TTS_API, TTS_VOICE, and TTS_PARAMS below to choose a provider.
    # If the selected provider fails or its API key is missing, speech falls back to gTTS.
    #
    # Google Cloud TTS (uses GOOGLE_APPLICATION_CREDENTIALS):
    #   TTS_API = 'google'
    #   TTS_VOICE = 'en-US-Journey-F'
    #   TTS_PARAMS = {'language_code': 'en-US'}
    #
    # ElevenLabs (uses ELEVENLABS_API_KEY):
    #   TTS_API = 'elevenlabs'
    #   TTS_VOICE = 'yowh82B72eMNrxcxHgBh' # Lorenzo Prada - Refined Italian accent 
    #   TTS_PARAMS = {
    #       'model_id': 'eleven_multilingual_v2',
    #       'output_format': 'mp3_44100_128',
    #       'voice_settings': {
    #           'stability': 0.5,
    #           'similarity_boost': 0.75,
    #       },
    #   }
    #
    # OpenAI (uses OPENAI_API_KEY):
    #   TTS_API = 'openai'
    #   TTS_VOICE = 'alloy'   # alloy/echo/fable/onyx/nova/shimmer
    #   TTS_PARAMS = {'model': 'gpt-4o-mini-tts'}
    #
    # Active selection:
    TTS_API = 'google'
    TTS_VOICE = 'en-US-Journey-F'
    TTS_PARAMS = {'language_code': 'en-US'}
    # ------------------------------------------------------------------------

    def __init__(self, robot):
        super().__init__(robot, 'sound')
        self.use_gcloud = True
        self.playing = False
        self.unpause_handle = None
        self.tts_client = None
        # ElevenLabs setup: read its key from the environment var.
        # The SDK client is created lazily on first use.
        self.eleven_api_key = os.getenv('ELEVENLABS_API_KEY')
        self.eleven_client = None
        # Google text to speech setup:
        try:
            creds = getattr(google.cloud, 'api_credentials', None)
            google_env = os.getenv('GOOGLE_APPLICATION_CREDENTIALS')
            # If no credentials, will look in GOOGLE_APPLICATION_CREDENTIALS environment var.
            if creds or google_env:
                self.tts_client = texttospeech.TextToSpeechClient(credentials = creds)
            self.tts_voice = texttospeech.VoiceSelectionParams(
                language_code="en-US",
                name="en-US-Journey-F",
                ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
            )
            self.tts_audio_config = texttospeech.AudioConfig(
                audio_encoding=texttospeech.AudioEncoding.MP3
            )
            synthesis_input = texttospeech.SynthesisInput(text="Hello")
            response = self.tts_client.synthesize_speech(
                input = synthesis_input,
                voice = self.tts_voice,
                audio_config = self.tts_audio_config
            )
        except Exception as e:
            print("Google text to speech:", e)
            self.tts_client = None
        # Cloud text-to-speech failed; use gTTs instead
        if self.tts_client is None:
            print('No Google Cloud credentials. Reverting to alternate speech synthesizer.')
            self.use_gcloud = False

    def status_update(self):
        if self.robot.robot0.sound.is_active():
            if not self.playing:
                self.playing = True
        else:  # sound is not active
            if self.playing is True:
                self.playing = False
                try:  # might fail if speech isn't up yet
                    self.unpause_handle = self.robot.loop.call_later(2, self.robot.speech_listener.unpause)
                except:
                    pass
                self.complete()

    def say_text(self, node, text):
        if self.robot.robot0.sound.is_active():
            print ('!!! SOUND ALREADY ACTIVE !!!')
        self.lock(node)
        if self.unpause_handle:
            self.unpause_handle.cancel()
            self.unpause_handle = None
        self.robot.loop.call_soon_threadsafe(self.launch_text_to_mp3, text)

    def launch_text_to_mp3(self, text):
        self.robot.loop.create_task(self.text_to_mp3(text))

    async def text_to_mp3(self, text):
        temp_dir = os.getenv('TEMP', '/tmp')
        speech_file_path = os.path.join(temp_dir, 'vex_speech.mp3')
        while True:
            self.synthesize_to_file(text, speech_file_path)
            self.robot.speech_listener.pause()
            try:
                self.robot.robot0.sound.play_local_file(speech_file_path, self.robot.sound_volume)
            except vex.aim.InvalidSoundFileException:   # file too long
                print("*** Speech too long. Truncating...")
                text = text[0:len(text)//2]
                continue
            return

    def get_tts_config(self):
        """Return the provider settings (api, voice, params) configured on this SoundActuator."""
        return self.TTS_API, self.TTS_VOICE, dict(self.TTS_PARAMS)

    def synthesize_to_file(self, text, speech_file_path):
        """Dispatch synthesis to the configured provider, with a gTTS safety net."""
        api, voice, params = self.get_tts_config()
        if api not in (None, 'google', 'elevenlabs', 'openai'):
            print(f'*** Unknown TTS_API {api!r}; using default Google/gTTS.')
            api = None
        try:
            if api == 'elevenlabs':
                self.synthesize_elevenlabs(text, speech_file_path, voice, params)
                return
            if api == 'openai':
                self.synthesize_openai(text, speech_file_path, voice, params)
                return
            # Default: Google Cloud when credentials are available.
            if self.tts_client is not None:
                self.synthesize_google(text, speech_file_path, voice, params)
                return
            # No Google credentials: fall through to the gTTS fallback below.
        except Exception as e:
            print(f'*** TTS provider ({api or "google"}) failed: {e}. Falling back to gTTS.')
        # Fallback synthesizer (also the normal path when no Google credentials).
        try:
            gTTS(text=text, lang='en').save(speech_file_path)
        except Exception as e:
            print(f'*** gTTS fallback failed: {e}')
            raise

    def synthesize_google(self, text, speech_file_path, voice=None, params=None):
        params = params or dict()
        voice_name = voice or params.get('voice') or self.tts_voice.name
        language_code = params.get('language_code', self.tts_voice.language_code)
        tts_voice = texttospeech.VoiceSelectionParams(
            language_code = language_code,
            name = voice_name,
        )
        synthesis_input = texttospeech.SynthesisInput(text=text)
        response = self.tts_client.synthesize_speech(
            input = synthesis_input,
            voice = tts_voice,
            audio_config = self.tts_audio_config
        )
        with open(speech_file_path, 'wb') as out:
            out.write(response.audio_content)

    def synthesize_elevenlabs(self, text, speech_file_path, voice, params):
        if not self.eleven_api_key:
            raise RuntimeError('No ELEVENLABS_API_KEY set in the environment')
        voice_id = voice or params.get('voice_id')
        if not voice_id:
            raise RuntimeError('No ElevenLabs voice id specified (TTS_VOICE)')
        if self.eleven_client is None:
            from elevenlabs.client import ElevenLabs
            self.eleven_client = ElevenLabs(api_key=self.eleven_api_key)
        model_id = params.get('model_id', 'eleven_multilingual_v2')
        output_format = params.get('output_format', 'mp3_44100_128')
        convert_kwargs = dict(
            text=text,
            voice_id=voice_id,
            model_id=model_id,
            output_format=output_format,
        )
        if 'voice_settings' in params:
            convert_kwargs['voice_settings'] = params['voice_settings']
        audio = self.eleven_client.text_to_speech.convert(**convert_kwargs)
        with open(speech_file_path, 'wb') as out:
            for chunk in audio:
                if chunk:
                    out.write(chunk)

    def synthesize_openai(self, text, speech_file_path, voice, params):
        client = getattr(getattr(self.robot, 'openai_client', None), 'client', None)
        if client is None:
            raise RuntimeError('OpenAI client unavailable (no OPENAI_API_KEY?)')
        model = params.get('model', 'gpt-4o-mini-tts')
        voice_name = voice or params.get('voice', 'alloy')
        with client.audio.speech.with_streaming_response.create(
            model=model, voice=voice_name, input=text
        ) as response:
            response.stream_to_file(speech_file_path)

    def play_sound(self, node, sound):
        self.lock(node)
        self.robot.robot0.sound.play(sound, self.robot.sound_volume)

    def play_sound_file(self, node, filepath):
        self.lock(node)
        self.robot.robot0.sound.play_local_file(filepath, self.robot.sound_volume)

    def play_note(self, node, pitch, duration):
        self.lock(node)
        self.robot.robot0.sound.play_note(pitch, duration, self.robot.sound_volume)


class KickActuator(Actuator):
    KICK_DURATION = 0.25 # seconds

    def __init__(self, robot):
        super().__init__(robot, 'kick')

    def kick(self, node, kicktype):
        self.lock(node)
        self.robot.robot0.kicker.kick(kicktype)
        self.robot.loop.call_soon_threadsafe(self.set_delayed_completion)

    def place(self, node):
        self.lock(node)
        self.robot.robot0.kicker.place()
        self.robot.loop.call_soon_threadsafe(self.set_delayed_completion)

    def set_delayed_completion(self):
        self.robot.loop.create_task(self.delayed_completion())

    async def delayed_completion(self):
        await asyncio.sleep(self.KICK_DURATION)
        if self.holder:
            self.holder.complete()


class LEDsActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'leds')
        self.NUM_LEDS = 6

    def stop(self):
        self.robot.robot0.led.on(vex.LightType.ALL_LEDS, vex.Color.TRANSPARENT)

    def set_light_color(self, node, *args):
        if len(args) == 2 or len(args) == 4:
            corrected_args = args
        else:
            corrected_args = [vex.LightType.ALL_LEDS, *args]
        self.lock(node)
        self.robot.robot0.led.on(*corrected_args)


class DisplayActuator(Actuator):
    EMOJI_NAMES =  [key for (key,value) in vars(vex.EmojiType).items()
                    if isinstance(value, vex.EmojiType)]

    EMOJI_VALUES = [v for v in vars(vex.EmojiType).values()
                    if isinstance(v, vex.EmojiType)]

    def __init__(self, robot):
        super().__init__(robot, 'display')

    def show_emoji(self, node, emoji, direction=vex.EmojiLookType.LOOK_FORWARD):
        self.lock(node)
        self.robot.robot0.screen.show_emoji(emoji, direction)
        self.current_emoji = emoji

    def hide_emoji(self, node):
        self.lock(node)
        self.robot.robot0.screen.hide_emoji()
        self.current_emoji = None
