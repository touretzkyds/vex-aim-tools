import os
import re
import cv2
import base64
import requests  # 替换 openai 为 requests
from dashscope import MultiModalConversation  # 阿里云多模态SDK

from .events import OpenAIEvent

default_preamble = """
  You are an intelligent mobile robot named Celeste.
  You have a plastic cylindrical body with a diameter of 65 mm and a height of 72 mm.
  You have three omnidirectional wheels and a forward-facing camera.
  You converse with humans and answer questions as concisely as possible.
  Pronounce "AprilTag-1.a" as "April Tag 1-A", and similarly for any word of form "AprilTag-N.x".
  Pronounce "OrangeBarrel.a" as "Orange Barrel A", pronounce "BlueBarrel.b" as "Blue Barrel B", and similarly for other barrel designators.
  Remember to be concise in your answers.
"""

class OpenAIClient():
    def __init__(self, robot, model='qwen-vl-max'):
        self.robot = robot
        self.model = model
        self.api_key = os.getenv("ALIYUN_API_KEY")
        if self.api_key:
            self.client = None
        else:
            print("*** No ALIYUN_API_KEY provided. 通义千问将不可用。")
            self.client = None
        self.set_preamble(default_preamble)

    def set_preamble(self, preamble):
        self.messages = [
            {'role': 'system', 'content': preamble}
        ]

    def query(self, query_text):
        system_prompt = self.robot.world_map.get_prompt()
        if not isinstance(system_prompt, str):
            print(f"[错误] system_prompt 不是字符串: {system_prompt} ({type(system_prompt)})")
            system_prompt = str(system_prompt or "")

        self.messages.append({'role': 'system', 'content': system_prompt})
        self.messages.append({'role': 'user', 'content': query_text})
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def camera_query(self, query_text):
        if self.robot.camera_image is None:
            print("[错误] camera_image 为 None，无法进行视觉查询。")
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        swapped_colors = cv2.cvtColor(self.robot.camera_image, cv2.COLOR_RGB2BGR)
        result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
        base64_image = base64.b64encode(encimg).decode('utf-8')

        self.messages.append({
            'role': 'user',
            'content': [
                {'image': base64_image},
                {'text': query_text}
            ]
        })
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def send_camera_image(self, instruction=None):
        if self.robot.camera_image is None:
            print("[错误] camera_image 为 None，无法发送图像。")
            return

        default_instruction = 'Here is the current camera image. Please go ahead and reply to the last request.'
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        swapped_colors = cv2.cvtColor(self.robot.camera_image, cv2.COLOR_RGB2BGR)
        result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
        base64_image = base64.b64encode(encimg).decode('utf-8')

        self.messages.append({
            'role': 'user',
            'content': [
                {'image': f'data:image/jpeg;base64,{base64_image}'},
                {'text': instruction or default_instruction}
            ]
        })
        self.robot.loop.call_soon_threadsafe(self.launch_openai_query)

    def smart_query(self, query_text):
        """
        根据 query_text 自动判断是否需要发送摄像头图像。
        如果是视觉相关问题，则调用 camera_query，否则调用 query。
        """
        visual_keywords = ["看见", "看到", "图像", "照片", "画面", "摄像头", "拍到", "视野"]
        if any(keyword in query_text for keyword in visual_keywords):
            print("[DEBUG] 触发视觉相关问题，使用 camera_query()")
            self.camera_query(query_text)
        else:
            print("[DEBUG] 非视觉问题，使用 query()")
            self.query(query_text)

    async def openai_query(self):
        if not self.api_key:
            return

        try:
            for msg in self.messages:
                content = msg.get("content")

                if isinstance(content, list):
                    for item in content:
                        if not isinstance(item, dict):
                            raise ValueError(f"[格式错误] content 中的项不是字典: {item} ({type(item)})")
                        if "image" in item and not isinstance(item["image"], str):
                            item["image"] = str(item["image"])
                        if "text" in item and not isinstance(item["text"], str):
                            item["text"] = str(item["text"])

                elif not isinstance(content, str):
                    raise ValueError(f"[格式错误] content 既不是字符串也不是多模态 list: {content} ({type(content)})")

            response = MultiModalConversation.call(
                model=self.model,
                messages=self.messages,
                api_key=self.api_key
            )

            answer = response["output"]["choices"][0]["message"]["content"]

            if isinstance(answer, list) and len(answer) > 0 and isinstance(answer[0], dict) and "text" in answer[0]:
                print("[修复] 从 answer list 中提取 text")
                answer = answer[0]["text"]

            self.messages.append({'role': 'assistant', 'content': answer})

            cleaned_answer = re.sub(r'\\[\[\]\(\)]', '', answer)
            event = OpenAIEvent(cleaned_answer)
            self.robot.erouter.post(event)

        except Exception as e:
            print(f"通义千问API错误: {e}")
            print("当前 messages:")
            for i, m in enumerate(self.messages):
                print(f"Message {i}: {m}")

    async def openai_oneshot_query(self, query_text, image=None):
        if not self.api_key:
            return

        content = [{'text': query_text}]
        if image is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            swapped_colors = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            result, encimg = cv2.imencode('.jpg', swapped_colors, encode_param)
            base64_image = base64.b64encode(encimg).decode('utf-8')
            content.insert(0, {'image': f'data:image/jpeg;base64,{base64_image}'})

        try:
            response = MultiModalConversation.call(
                model=self.model,
                messages=[{'role': 'user', 'content': content}],
                api_key=self.api_key
            )
            answer = response["output"]["choices"][0]["message"]["content"]
            cleaned_answer = re.sub(r'\\[\[\]\(\)]', '', answer)
            event = OpenAIEvent(cleaned_answer)
            self.robot.erouter.post(event)
        except Exception as e:
            print(f"通义千问One-Shot错误: {e}")

    def launch_openai_query(self):
        self.robot.loop.create_task(self.openai_query())

    def oneshot_query(self, query_text, image=None):
        self.robot.loop.call_soon_threadsafe(self.launch_openai_oneshot_query, query_text, image)

    def launch_openai_oneshot_query(self, query_text, image=None):
        self.robot.loop.create_task(self.openai_oneshot_query(query_text, image))
