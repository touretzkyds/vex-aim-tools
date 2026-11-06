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
  Remember to be concise in your answers, but warm and friendly.
  Prefer one clear spoken sentence; offer more detail only if the user asks.
"""

class OpenAIClient():
    # DEFAULT_MODEL = 'gpt-4o'
    DEFAULT_MODEL = 'gpt-5.5'
    VECTOR_STORE_EXPIRY_DAYS = 1
    MAX_SEARCH_RESULTS = 5
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
        self._store_id = None
        self.vector_store_id = None
        self.documents = []
        self.set_preamble(default_preamble)

    def set_preamble(self, preamble):
        self.messages = [
            {'role': 'system', 'content': preamble}
        ]
        self.pinned_count = 1
        for filename in list(getattr(self, 'documents', ())):
            self._pin_note_text(filename)

    def query(self, query_text):
        self.messages.append({'role': 'system', 'content': self.robot.world_map.get_prompt()})
        self.messages.append({'role': 'user', 'content': query_text})
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def note_for_later(self, text):
        self.messages.append({'role': 'system', 'content': text})

    def _ensure_vector_store(self):
        "Create this session's store on first use; one store holds every document."
        if self._store_id is None:
            store = self.client.vector_stores.create(
                name='celeste-docs',
                expires_after={'anchor': 'last_active_at',
                               'days': self.VECTOR_STORE_EXPIRY_DAYS},
            )
            self._store_id = store.id
        return self._store_id

    def attach_pdf(self, filename, data):
        """Index a PDF for retrieval instead of attaching it to every turn.
        Called from the Flask upload thread; blocks it until indexing finishes."""
        if self.client is None:
            raise RuntimeError("OpenAI is not configured")
        store_id = self._ensure_vector_store()
        vs_file = self.client.vector_stores.files.upload_and_poll(
            vector_store_id=store_id,
            file=(filename, data, 'application/pdf'),
        )
        if vs_file.status != 'completed':
            raise RuntimeError('OpenAI could not index this PDF (%s)'
                               % (vs_file.last_error or vs_file.status,))
        self.vector_store_id = store_id
        self.robot.loop.call_soon_threadsafe(self._pin_document_note, filename)
        return store_id

    def _pin_note_text(self, filename):
        """Tell the model the document exists, ahead of the trimmed history.
        file_search is a tool the model chooses to call; with no note it has no
        reason to think there is anything to search and answers from its own
        knowledge instead."""
        self.messages.insert(self.pinned_count,
            {'role': 'system',
             'content': 'A document named "%s" has been loaded. Use the '
                        'file_search tool to look up anything the user asks '
                        'about its contents.' % filename})
        self.pinned_count += 1

    def _pin_document_note(self, filename):
        self.documents.append(filename)
        self._pin_note_text(filename)

    def _tools(self):
        "Built per call: the vector store does not exist until a document is uploaded."
        if not self.vector_store_id:
            return []
        return [{'type': 'file_search',
                 'vector_store_ids': [self.vector_store_id],
                 'max_num_results': self.MAX_SEARCH_RESULTS}]

    def camera_query(self, query_text):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        swapped_colors = cv2.cvtColor(self.robot.camera_image, cv2.COLOR_RGB2BGR)
        result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
        base64_image = base64.b64encode(encimg).decode('utf-8')
        self.messages.append(
            {'role' : 'user',
             'content' : [
                 {'type': 'input_text', 'text': query_text },
                 {'type': 'input_image',
                  'image_url': f'data:image/jpeg;base64,{base64_image}'}
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
                 {'type': 'input_text', 'text': instruction or default_instruction},
                 {'type': 'input_image',
                  'image_url': f'data:image/jpeg;base64,{base64_image}'}
             ]})
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def launch_openai_query(self):
        self.robot.loop.create_task(self.openai_query())

    def _trim_history(self, max_messages=200):
        """
        Keeps the system prompt (index 0) and the last `max_messages`
        from the history, to prevent context window overflow.
        """
        keep = self.pinned_count
        if len(self.messages) > (max_messages + keep):
            self.messages = self.messages[:keep] + self.messages[-max_messages:]

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
                    if part.get('type') == 'input_text':
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
            response = self.client.responses.create(
                model = self.model,
                input = list(self.messages),
                tools = self._tools(),
            )
            answer = response.output_text
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
        content = [ {'type': 'input_text', 'text': query_text } ]
        if image is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            swapped_colors = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
            base64_image = base64.b64encode(encimg).decode('utf-8')
            content.append({'type': 'input_image',
                            'image_url': f'data:image/jpeg;base64,{base64_image}'})
        messages = [ {'role': 'user',
                      'content': content } ]
        response = self.client.responses.create(
            model = self.model,
            input = messages,
        )
        answer = response.output_text
        cleaned_answer = re.sub(r'\\[\[\]\(\)]', '', answer)
        event = OpenAIEvent(cleaned_answer)
        self.robot.erouter.post(event)
