import os
import re
import cv2
import base64
import openai

from .events import OpenAIEvent

default_preamble = """
  You are an intelligent mobile robot named Celeste.
  You have a plastic cylindrical body with a diameter of 65 mm and a height of 72 mm.
  You have three omnidirectional wheels and a forward-facing camera.
  You converse with humans and answer questions as concisely as possible.
  Pronounce "AprilTag-1.a" as "April Tag 1-A", and similarly for any word of form "AprilTag-N.x".
  Pronounce "OrangeBarrel.a" as "Orange Barrel A", pronounce "BlueBarrel.b" as "Blue Barrel B", and similarly for other barrel designators.
  Remember to be concise in your answers.
  Prefer one clear spoken sentence; offer more detail only if the user asks.
"""

class OpenAIClient():
    DEFAULT_MODEL = 'gpt-4o'
    def __init__(self, robot, model=DEFAULT_MODEL, use_moderation=False):
        self.robot = robot
        self.model = model
        self.use_moderation = use_moderation
        env_key = os.getenv("OPENAI_API_KEY")
        if env_key:
            openai.api_key = env_key
        if openai.api_key:  # may have been set by parent program if not by env_key
            self.client = openai.OpenAI(api_key = openai.api_key)
        else:
            print("*** No OPENAI_API_KEY provided.  GPT will not be available.")
            self.client = None
        self.set_preamble(default_preamble)

    def set_preamble(self, preamble):
        self.messages = [
            {'role': 'system', 'content': preamble}
        ]

    def query(self, query_text):
        self.messages.append({'role': 'system', 'content': self.robot.world_map.get_prompt()})
        self.messages.append({'role': 'user', 'content': query_text})
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def note_for_later(self, text):
        self.messages.append({'role': 'system', 'content': text})

    def camera_query(self, query_text):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        swapped_colors = cv2.cvtColor(self.robot.camera_image, cv2.COLOR_RGB2BGR)
        result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
        base64_image = base64.b64encode(encimg).decode('utf-8')
        self.messages.append(
            {'role' : 'user',
             'content' : [
                 {'type': 'text', 'text': query_text },
                 {'type': 'image_url',
                  'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'}}
             ]})
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def send_camera_image(self, instruction=None):
        default_instruction = 'Here is the current camera image. Please go ahead and reply to the last request.' 
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        swapped_colors = cv2.cvtColor(self.robot.camera_image, cv2.COLOR_RGB2BGR)
        result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
        base64_image = base64.b64encode(encimg).decode('utf-8')
        self.messages.append(
            {'role' : 'user',
             'content' : [
                 {'type': 'text', 'text': instruction or default_instruction},
                 {'type': 'image_url',
                  'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'}}
             ]})
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def launch_openai_query(self):
        self.robot.loop.create_task(self.openai_query())

    def _trim_history(self, max_messages=200):
        """
        Keeps the system prompt (index 0) and the last `max_messages`
        from the history, to prevent context window overflow.
        """
        if len(self.messages) > (max_messages + 1):
            # Preserves the preamble (self.messages[0])
            # and appends the last `max_messages` from the history.
            self.messages = [self.messages[0]] + self.messages[-(max_messages):]

    async def _moderate_text(self, text):
        """
        Calls the OpenAI Moderation API.
        Returns True if flagged, False otherwise.
        Fails safe (returns True) on error.
        """
        if not text:
            return False  # Do not flag empty strings
        try:
            response = self.client.moderations.create(
                model="omni-moderation-latest",
                input=text
            )
            result = response.results[0]
            return result.flagged
        except Exception as e:
            print(f"*** Moderation API call failed: {e}. Failing safe.")
            return True  # Fail-safe: assume text is flagged if API fails

    async def openai_query(self):
        if self.client is None:
            return

        # --- 1. Moderate User Input ---
        user_query_text = ""
        # Find the last user message to moderate it
        if self.messages and self.messages[-1]['role'] == 'user':
            user_query_content = self.messages[-1]['content']
            # Handle both string and list content (for images)
            if isinstance(user_query_content, list):
                # Find the text part in the list
                for part in user_query_content:
                    if part.get('type') == 'text':
                        user_query_text = part.get('text', '')
                        break
            elif isinstance(user_query_content, str):
                user_query_text = user_query_content

        if user_query_text and self.use_moderation:
            print('moderate input')
            user_flagged = await self._moderate_text(user_query_text)
            if user_flagged:
                print("*** User input flagged by moderation.")
                # Remove the flagged user message.
                self.messages.pop()
                # Also remove the system world_map prompt that preceded it.
                if self.messages and self.messages[-1]['role'] == 'system':
                    self.messages.pop()
                
                # Post a canned, safe response and stop.
                safe_answer = "I'm sorry, I can't talk about that topic."
                event = OpenAIEvent(safe_answer)
                self.robot.erouter.post(event)
                return

        # --- 2. Trim History ---
        # Call trim_history *after* user check, *before* API call.
        self._trim_history()

        # --- 3. Call Completion API ---
        try:
            response = self.client.chat.completions.create(
                model = self.model,
                messages = self.messages
            )
            answer = response.choices[0].message.content
        except Exception as e:
            print(f"*** OpenAI completion call failed: {e}")
            # Post a generic error and stop.
            safe_answer = "I'm sorry, I had trouble generating a response."
            event = OpenAIEvent(safe_answer)
            self.robot.erouter.post(event)
            return

        # --- 4. Moderate Assistant Output ---
        if self.use_moderation:
            print('moderate output')
            assistant_flagged =  await self._moderate_text(answer)
        else:
            assistant_flagged = False
        if assistant_flagged:
            print("*** Assistant output flagged by moderation.")
            # Do NOT append the flagged answer to history.
            # Post a canned, safe response and stop.
            safe_answer = "I'm sorry, I can't generate a response about that."
            event = OpenAIEvent(safe_answer)
            self.robot.erouter.post(event)
            return

        # --- 5. Process Good Response ---
        # If both checks pass, append the good answer to history.
        self.messages.append({'role': 'assistant', 'content': answer})
        
        # remove LaTeX brackets from response
        cleaned_answer = re.sub(r'\\[\[\]\(\)]', '', answer)
        event = OpenAIEvent(cleaned_answer)
        self.robot.erouter.post(event)

    # One-shot version doesn't use preamble or maintain message history

    def oneshot_query(self, query_text, image=None):
        self.robot.loop.call_soon_threadsafe(self.launch_openai_oneshot_query, query_text, image)


    def launch_openai_oneshot_query(self, query_text, image=None):
        self.robot.loop.create_task(self.openai_oneshot_query(query_text, image))

    async def openai_oneshot_query(self, query_text, image=None):
        if self.client is None:
            return
        content = [ {'type': 'text', 'text': query_text } ]
        if image is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            swapped_colors = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
            base64_image = base64.b64encode(encimg).decode('utf-8')
            content.append({'type': 'image_url',
                            'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'}})
        messages = [ {'role': 'user',
                      'content': content } ]
        response = self.client.chat.completions.create(
            model = self.model,
            messages = messages
        )
        answer = response.choices[0].message.content
        cleaned_answer = re.sub(r'\\[\[\]\(\)]', '', answer)
        event = OpenAIEvent(cleaned_answer)
        self.robot.erouter.post(event)
