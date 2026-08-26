import asyncio
import os
import math
import time
from math import pi, sin, cos, atan2

from gtts import gTTS
import google.cloud
from google.cloud import texttospeech

import vex
from .geometry import wrap_angle
from .speech_chunking import chunk_speech_text, force_split_chunk
from .speech_rec import add_to_transcript

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
        # Multi-chunk say pipeline state (text_to_mp3)
        self.speech_pipeline_active = False
        self._speech_cancelled = False
        self.play_finished = None
        self._speech_task = None
        # Generation counters so we only accept active-to-inactive for the
        # chunk we just started, to avoid waking early or having overlapped plays.
        self._play_gen = 0
        self._active_play_gen = 0
        self._chunk_seen_active_gen = 0
        self._play_armed_at = 0.0
        # Ignore finish edges in this window after play_local_file (stale status).
        self._min_play_s = 0.25
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

    def clear(self):
        self._cancel_speech_pipeline(cancel_task=True)
        super().clear()

    def unlock(self, node):
        abort_pipeline = (self.holder is node and self.speech_pipeline_active)
        super().unlock(node)
        if abort_pipeline:
            # Say node stopped early; wake the pipeline without cancelling
            # the task from inside a normal complete()->unlock path
            self._cancel_speech_pipeline(cancel_task=False)

    def unlock_if_held(self, node):
        abort_pipeline = (self.holder is node and self.speech_pipeline_active)
        super().unlock_if_held(node)
        if abort_pipeline:
            self._cancel_speech_pipeline(cancel_task=False)

    def _cancel_speech_pipeline(self, cancel_task=True):
        """Stop current multi-chunk speak; not complete."""
        self._speech_cancelled = True
        self.speech_pipeline_active = False
        if cancel_task and self._speech_task is not None and not self._speech_task.done():
            self._speech_task.cancel()
        event = self.play_finished
        if event is not None:
            try:
                self.robot.loop.call_soon_threadsafe(event.set)
            except Exception:
                try:
                    event.set()
                except Exception:
                    pass

    def status_update(self):
        if self.robot.robot0.sound.is_active():
            if not self.playing:
                self.playing = True
            if self.speech_pipeline_active and self._active_play_gen:
                # Record that the current chunk was observed playing successfully
                self._chunk_seen_active_gen = self._active_play_gen
        else:  # sound is not active
            if self.playing is True:
                self.playing = False
                if self.speech_pipeline_active:
                    # Only wake after this generation was seen active, then idle.
                    # Ignore finishes that arrive immediately after arming a play.
                    # This problem caused silent/cut-off speech.
                    armed_age = time.monotonic() - self._play_armed_at
                    if (self.play_finished is not None and
                            self._chunk_seen_active_gen == self._active_play_gen and
                            self._active_play_gen != 0 and
                            armed_age >= self._min_play_s):
                        self.play_finished.set()
                    return
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
        self._speech_cancelled = False
        add_to_transcript(self.robot.character_name + ": " + text)
        self.robot.loop.call_soon_threadsafe(self.launch_text_to_mp3, text)

    def launch_text_to_mp3(self, text):
        self._speech_task = self.robot.loop.create_task(self.text_to_mp3(text))

    async def _synthesize_async(self, text, speech_file_path):
        """Run blocking TTS in a thread so playback can overlap with prefetch."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None, self.synthesize_to_file, text, speech_file_path
        )

    def _speech_aborted(self):
        return self._speech_cancelled or self.holder is None

    async def _wait_until_sound_idle(self, timeout_s=5.0):
        """Wait until robot sound is inactive. Never hang on a stale playing flag."""
        remaining = timeout_s
        while remaining > 0:
            if self._speech_aborted():
                return
            if not self.robot.robot0.sound.is_active():
                self.playing = False
                return
            await asyncio.sleep(0.02)
            remaining -= 0.02
        # Timed out: clear local flag and proceed so we never block speech forever.
        if self.robot.robot0.sound.is_active():
            print('*** Sound still active before next chunk; proceeding anyway.')
        self.playing = False

    async def _wait_for_chunk_playback(self, play_gen):
        """Wait for this chunk to start after arming, then end. 
        Don't treat the ones not started yet or old active as done.
        """
        warned = False
        elapsed = 0.0
        # observe THIS generation become active after play_local_file.
        while not self._speech_aborted():
            if self._chunk_seen_active_gen == play_gen:
                break
            # Ultra-short clip: finish already signaled for this gen.
            if (self.play_finished.is_set() and
                    self._chunk_seen_active_gen == play_gen):
                return True
            await asyncio.sleep(0.01)
            elapsed += 0.01
            if not warned and elapsed >= 10.0:
                print('*** Still waiting for speech chunk to become active...')
                warned = True
            # Fallback: if robot never toggles is_active but enough time passed since arming, 
            # poll is_active once more and accept a late rising edge.
            if (elapsed >= self._min_play_s and
                    self.robot.robot0.sound.is_active() and
                    self._active_play_gen == play_gen):
                self._chunk_seen_active_gen = play_gen
                self.playing = True
                break
        else:
            return False

        if self._speech_aborted():
            return False

        # Wait for active-to-inactive for this generation.
        # If status_update ignored an early finish edge inside_min_play_s,
        # we still complete once the robot is idle after that window.
        while not self._speech_aborted():
            if self.play_finished.is_set():
                return True
            if (self._chunk_seen_active_gen == play_gen and
                    not self.robot.robot0.sound.is_active() and
                    (time.monotonic() - self._play_armed_at) >= self._min_play_s):
                return True
            await asyncio.sleep(0.01)
        return False

    async def text_to_mp3(self, text):
        """Split long text, TTS with two buffers, complete only when all are done."""
        chunks = chunk_speech_text(text)
        if not chunks:
            if not self._speech_aborted():
                self.complete()
            return

        temp_dir = os.getenv('TEMP', '/tmp')
        buffers = [
            os.path.join(temp_dir, 'vex_speech_a.mp3'),
            os.path.join(temp_dir, 'vex_speech_b.mp3'),
        ]

        self.speech_pipeline_active = True
        self.play_finished = asyncio.Event()
        self._play_gen = 0
        self._active_play_gen = 0
        self._chunk_seen_active_gen = 0
        self.playing = False
        prefetch_task = None

        try:
            self.robot.speech_listener.pause()
            await self._synthesize_async(chunks[0], buffers[0])
            if self._speech_aborted():
                return

            i = 0
            while i < len(chunks):
                if self._speech_aborted():
                    return

                buf = buffers[i % 2]
                next_buf = buffers[(i + 1) % 2]

                # Prefetch next chunk into the other buffer while this one plays
                prefetch_task = None
                if i + 1 < len(chunks):
                    prefetch_task = asyncio.create_task(
                        self._synthesize_async(chunks[i + 1], next_buf)
                    )

                # Ensure previous audio is fully idle before starting this chunk
                await self._wait_until_sound_idle()
                if self._speech_aborted():
                    if prefetch_task is not None:
                        prefetch_task.cancel()
                        try:
                            await prefetch_task
                        except asyncio.CancelledError:
                            pass
                    return

                try:
                    self._play_gen += 1
                    play_gen = self._play_gen
                    self.play_finished.clear()
                    self.playing = False
                    self._chunk_seen_active_gen = 0
                    # Arm generation only when issuing play;
                    # add timestamp so stale inactive edges right after arming are directly ignored.
                    self._active_play_gen = play_gen
                    self._play_armed_at = time.monotonic()
                    self.robot.robot0.sound.play_local_file(
                        buf, self.robot.sound_volume
                    )
                except vex.aim.InvalidSoundFileException:
                    # Last-resort: split this chunk further and retry; no text loss.
                    self._active_play_gen = 0
                    if prefetch_task is not None:
                        prefetch_task.cancel()
                        try:
                            await prefetch_task
                        except asyncio.CancelledError:
                            pass
                        prefetch_task = None
                    parts = force_split_chunk(chunks[i])
                    if len(parts) <= 1:
                        # Cannot split further; drop to a hard half-cut as emergency fallback.
                        mid = max(1, len(chunks[i]) // 2)
                        parts = [chunks[i][:mid], chunks[i][mid:]]
                        parts = [p for p in parts if p]
                    print(
                        f'*** Speech chunk too long ({len(chunks[i])} chars). '
                        f'Splitting into {len(parts)} pieces and retrying...'
                    )
                    chunks = chunks[:i] + parts + chunks[i + 1:]
                    await self._synthesize_async(chunks[i], buf)
                    if self._speech_aborted():
                        return
                    continue

                finished_ok = await self._wait_for_chunk_playback(play_gen)
                self._active_play_gen = 0
                if self._speech_aborted() or not finished_ok:
                    if prefetch_task is not None:
                        prefetch_task.cancel()
                        try:
                            await prefetch_task
                        except asyncio.CancelledError:
                            pass
                    return

                if prefetch_task is not None:
                    await prefetch_task
                    prefetch_task = None

                i += 1

            if not self._speech_aborted():
                try:
                    self.unpause_handle = self.robot.loop.call_later(
                        2, self.robot.speech_listener.unpause
                    )
                except Exception:
                    pass
                # Clear pipeline flag before complete() so unlock-on-complete is not seen as an early abort
                self.speech_pipeline_active = False
                self.complete()
        except asyncio.CancelledError:
            self._speech_cancelled = True
            raise
        finally:
            self.speech_pipeline_active = False
            self._active_play_gen = 0
            if prefetch_task is not None and not prefetch_task.done():
                prefetch_task.cancel()
                try:
                    await prefetch_task
                except (asyncio.CancelledError, Exception):
                    pass

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
