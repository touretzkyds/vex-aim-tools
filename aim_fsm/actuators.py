import asyncio
import os
import hashlib
import time
import requests
from math import pi

from gtts import gTTS

from . import aim
from . import vex

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
            raise self.ActuatorLocked(self)

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

    def stop(self):
        self.robot.robot0.stop_all_movement()

    def status_update(self):
        # Bad timing can cause a just-started node to complete prematurely;
        # must wait until robot is seen to be moving before considering
        # looking for a stopped-moving status.
        if self.robot.robot0.is_move_active() or self.robot.robot0.is_turn_active():
            self.started = True
        elif self.holder and self.started:
            self.holder.complete()
            self.holder = None
            self.started = False

    def turn(self, node, angle_rads, turn_speed=None):
        self.lock(node)
        self.started = False
        if angle_rads > 0:
            turntype = vex.TurnType.LEFT
        else:
            turntype = vex.TurnType.RIGHT
        self.robot.robot0.turn_for(turntype, abs(angle_rads)*180/pi, turn_speed=turn_speed, wait=False)

    def forward(self, node, distance_mm, drive_speed=None):
        self.lock(node)
        self.started = False
        angle_forward = 0
        self.robot.robot0.move_for(distance_mm, angle_forward, drive_speed=drive_speed, wait=False)

    def sideways(self, node, distance_mm, drive_speed=None):
        self.lock(node)
        self.started = False
        angle_leftward = -90
        self.robot.robot0.move_for(distance_mm, angle_leftward, drive_speed=drive_speed, wait=False)

    def move(self, node, distance_mm, angle_rads, drive_speed=None, turn_speed=None):
        self.lock(node)
        self.started = False
        self.robot.robot0.move_for(distance_mm, angle_rads*180/pi,
                                   drive_speed=drive_speed, turn_speed=turn_speed, wait=False)
""" 
class SoundActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'sound')
        self.use_baidu = True
        self.playing = False
        
        # Baidu TTS setup
        self.baidu_app_id = os.getenv('BAIDU_APP_ID')
        self.baidu_api_key = os.getenv('BAIDU_TTS_API_KEY')
        self.baidu_secret_key = os.getenv('BAIDU_SECRET_KEY')
        
        if not all([self.baidu_app_id, self.baidu_api_key, self.baidu_secret_key]):
            print('Baidu TTS credentials not found. Reverting to alternate speech synthesizer.')
            self.use_baidu = False

    def status_update(self):
        if self.robot.robot0.is_sound_active():
            self.playing = True
        else:
            if self.playing is True:
                self.playing = False
                try:  # might fail if speech isn't up yet
                    self.robot.loop.call_later(1, self.robot.speech_listener.unpause)
                except:
                    pass
                self.complete()

    def say_text(self, node, text):
        self.lock(node)
        self.robot.loop.call_soon_threadsafe(self.launch_text_to_mp3, text)

    def launch_text_to_mp3(self, text):
        self.robot.loop.create_task(self.text_to_mp3(text))

    async def text_to_mp3(self, text):
        temp_dir = os.getenv('TEMP', '/tmp')
        speech_file_path = os.path.join(temp_dir, 'vex_speech.mp3')
        
        while True:
            if self.use_baidu:
                try:
                    # Generate Baidu TTS token
                    token = await self._get_baidu_token()
                    if not token:
                        raise Exception("Failed to get Baidu token")
                    
                    # Call Baidu TTS API
                    tts_url = f"https://tsn.baidu.com/text2audio?tex={text}&tok={token}&cuid=vexrobot&ctp=1&lan=zh&per=1"
                    response = requests.get(tts_url)
                    
                    if response.headers['Content-Type'] != 'audio/mp3':
                        raise Exception("Baidu TTS failed: " + response.text)
                    
                    with open(speech_file_path, 'wb') as out:
                        out.write(response.content)
                except Exception as e:
                    print(f"Baidu TTS failed: {e}. Falling back to gTTS")
                    self.use_baidu = False
                    continue
            else:
                tts = gTTS(text=text, lang='en')
                tts.save(speech_file_path)
            
            self.robot.speech_listener.pause()
            try:
                self.robot.robot0.play_sound_file(speech_file_path)
            except aim.invalid_sound_file_exception:   # file too long
                print("*** Speech too long. Truncating...")
                text = text[0:len(text)//2]
                continue
            return

    async def _get_baidu_token(self):
        auth_url = "https://openapi.baidu.com/oauth/2.0/token"
        params = {
            'grant_type': 'client_credentials',
            'client_id': self.baidu_api_key,
            'client_secret': self.baidu_secret_key
        }
        
        try:
            response = requests.get(auth_url, params=params)
            data = response.json()
            return data.get('access_token')
        except Exception as e:
            print(f"Error getting Baidu token: {e}")
            return None

    def play_sound(self, node, sound, volume=100):
        self.lock(node)
        self.robot.robot0.play_sound(sound, volume)

    def play_sound_file(self, node, filepath):
        self.lock(node)
        self.robot.robot0.play_sound_file(filepath)
 """
class SoundActuator(Actuator):
    def __init__(self, robot):
            super().__init__(robot, 'sound')
            self.use_baidu = True
            self.playing = False
            
            # 百度TTS音色参数配置
            self.tts_settings = {
                'voice_type': 4,      # 发音人：0-女声，1-男声，3-情感男声，4-情感女声
                'speed': 5,           # 语速：0-15，默认5
                'pitch': 5,           # 音调：0-15，默认5
                'volume': 5           # 音量：0-15，默认5
            }
            
            # Baidu TTS setup
            self.baidu_app_id = os.getenv('BAIDU_APP_ID')
            self.baidu_api_key = os.getenv('BAIDU_TTS_API_KEY')
            self.baidu_secret_key = os.getenv('BAIDU_SECRET_KEY')
            
            if not all([self.baidu_app_id, self.baidu_api_key, self.baidu_secret_key]):
                print('Baidu TTS credentials not found. Reverting to alternate speech synthesizer.')
                self.use_baidu = False

    def set_voice_style(self, voice_type=None, speed=None, pitch=None, volume=None):
            """设置语音风格
            Args:
                voice_type: 发音人类型 
                    0-女声，1-男声，3-情感男声，4-情感女声
                speed: 语速 (0-15)
                pitch: 音调 (0-15)
                volume: 音量 (0-15)
            """
            if voice_type is not None:
                self.tts_settings['voice_type'] = voice_type
            if speed is not None:
                self.tts_settings['speed'] = speed
            if pitch is not None:
                self.tts_settings['pitch'] = pitch
            if volume is not None:
                self.tts_settings['volume'] = volume

    def status_update(self):
        if self.robot.robot0.is_sound_active():
            self.playing = True
        else:
            if self.playing is True:
                self.playing = False
                try:  # might fail if speech isn't up yet
                    self.robot.loop.call_later(1, self.robot.speech_listener.unpause)
                except:
                    pass
                self.complete()

    def say_text(self, node, text):
        self.lock(node)
        self.robot.loop.call_soon_threadsafe(self.launch_text_to_mp3, text)

    def launch_text_to_mp3(self, text):
        self.robot.loop.create_task(self.text_to_mp3(text))

    async def text_to_mp3(self, text):
        temp_dir = os.getenv('TEMP', '/tmp')
        speech_file_path = os.path.join(temp_dir, 'vex_speech.mp3')
        
        while True:
            if self.use_baidu:
                try:
                    token = await self._get_baidu_token()
                    if not token:
                        raise Exception("Failed to get Baidu token")
                    
                    # 构建带音色参数的TTS请求
                    tts_url = (
                        f"https://tsn.baidu.com/text2audio?"
                        f"tex={requests.utils.quote(text)}&"
                        f"tok={token}&"
                        f"cuid=vexrobot&"
                        f"ctp=1&"
                        f"lan=zh&"
                        f"per={self.tts_settings['voice_type']}&"
                        f"spd={self.tts_settings['speed']}&"
                        f"pit={self.tts_settings['pitch']}&"
                        f"vol={self.tts_settings['volume']}"
                    )
                    
                    response = requests.get(tts_url)
                    
                    if response.headers.get('Content-Type', '') != 'audio/mp3':
                        raise Exception("Baidu TTS failed: " + response.text[:200])
                    
                    with open(speech_file_path, 'wb') as out:
                        out.write(response.content)
                        
                except Exception as e:
                    print(f"Baidu TTS failed: {e}. Falling back to gTTS")
                    self.use_baidu = False
                    continue
            else:
                tts = gTTS(text=text, lang='zh-cn' if self.tts_settings['voice_type'] in [0,4] else 'en')
                tts.save(speech_file_path)
            
            self.robot.speech_listener.pause()
            try:
                self.robot.robot0.play_sound_file(speech_file_path)
            except aim.invalid_sound_file_exception:
                print("*** Speech too long. Truncating...")
                text = text[0:len(text)//2]
                continue
            return

    async def _get_baidu_token(self):
        auth_url = "https://openapi.baidu.com/oauth/2.0/token"
        params = {
            'grant_type': 'client_credentials',
            'client_id': self.baidu_api_key,
            'client_secret': self.baidu_secret_key
        }
        
        try:
            response = requests.get(auth_url, params=params)
            data = response.json()
            return data.get('access_token')
        except Exception as e:
            print(f"Error getting Baidu token: {e}")
            return None

    def play_sound(self, node, sound, volume=100):
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
            self.holder.complete()


class LEDsActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'leds')
        self.NUM_LEDS = 6

    def stop(self):
        self.robot.robot0.clear_leds()

    def set_light_color(self, node, *args):
        self.lock(node)
        self.robot.robot0.set_light_color(*args)