from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import webbrowser
import threading
import os
import sys
import logging

try:
    from playsound3 import playsound
except: pass

from .thesaurus import Thesaurus
from .evbase import Event
from .events import SpeechEvent


app = Flask('VEX AIM Speech Listener')
CORS(app)  # This will enable CORS for all routes

session_id = None

@app.route('/')
def serve_index():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(this_dir, '..' )
    return send_from_directory(parent_dir, 'speech_listener.html')

@app.route('/closed.html')
def serve_closed():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(this_dir, '..' )
    return send_from_directory(parent_dir, 'listener_closed.html')

@app.route('/api/set-session-id', methods=['POST'])
def handle_set_session_id():
    global session_id
    data = request.json
    session_id = data['sessionID']
    return jsonify({'status': 'ok'})

@app.route('/api/get-session-id', methods=['POST'])
def handle_get_session_id():
    global session_id
    return jsonify({'sessionID': session_id})

@app.route('/api/speech-to-text', methods=['POST'])
def handle_speech_to_text():
    global speech_listener
    data = request.json
    speech_listener.handle_utterance(data['text'])
    return jsonify({'status': 'ok'})

class SpeechListener():
    def __init__(self, _robot, thesaurus=Thesaurus(), debug=False, confirmation_bell=False):
        global robot, speech_listener
        robot = _robot
        speech_listener = self

        self.robot = robot
        self.confirmation_bell = confirmation_bell
        self.thesaurus = thesaurus
        self.debug = debug
        self.enabled = True
        self.paused = False  # speaking pauses the listener

    def run_flask(self):
        # Suppress the default Flask logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        # Debug must be false to prevent duplicate tab:
        app.run(port=51327, debug=False, use_reloader=False)

    def load_listener_page(self):
        webbrowser.open_new_tab('http://127.0.0.1:51327/')
        print('Speech input is running in your browser.')

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True
        self.paused = False

    def pause(self):
        self.paused = True
        #print('Speech paused')

    def unpause(self):
        self.paused = False
        #print('Speech unpaused')

    def set_confirmation(self, value=True):
        self.confirmation_bell = value

    def handle_utterance(self, utterance):
        if not self.enabled or len(utterance) == 0:
            return
        elif self.paused:
            print('Discarded:', utterance, flush=True)
            return
        print("Raw utterance: '%s'" % utterance)
        utterance = utterance.strip().lower()
        words = [self.thesaurus.lookup_word(w) for w in utterance.split(" ")]
        words = self.thesaurus.substitute_phrases(words)
        string = " ".join(words)
        print("Heard: '%s'" % string)
        sys.stdout.flush()
        if len(string) == 0:
            return
        if self.confirmation_bell:
            #playsound(os.path.abspath("media/yaru-bell.mp3"))
            playsound(os.path.abspath("media/acknowledge4.mp3"))
        event = SpeechEvent(string, words)
        self.robot.erouter.post(event)
        
    def start(self):
        if self.robot.flask_thread:
            return
        self.robot.flask_thread = threading.Thread(target=self.run_flask)
        self.robot.flask_thread.start()
        self.robot.loop.call_later(1, self.load_listener_page)
