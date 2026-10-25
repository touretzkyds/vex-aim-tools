from flask import Flask, request, jsonify, send_from_directory, Response
from flask_cors import CORS
import webbrowser
import threading
import queue
import os
import sys
import pathlib
import logging
import time
from playsound3 import playsound

from .thesaurus import Thesaurus
from .document_upload import DocumentError, prepare_document
from .evbase import Event
from .events import SpeechEvent


LISTENER_PORT = 51327
ALLOWED_HOSTS = {'127.0.0.1:%d' % LISTENER_PORT, 'localhost:%d' % LISTENER_PORT}
ALLOWED_ORIGINS = {'http://' + host for host in ALLOWED_HOSTS}

app = Flask('VEX AIM Speech Listener')
# Only our own page may read responses (127.0.0.1 and localhost count as
# different origins, so both are listed).
CORS(app, origins=list(ALLOWED_ORIGINS))

@app.before_request
def reject_foreign_requests():
    # CORS only stops other sites from reading responses; browsers still send
    # simple cross-site POSTs (e.g. reset-fsm, file uploads).  So refuse any
    # request a browser marks as coming from another origin, and any request
    # addressed to another host name (DNS rebinding).  Requests without an
    # Origin header come from same-origin GETs or local non-browser tools.
    if request.host not in ALLOWED_HOSTS:
        return jsonify({'status': 'error', 'error': 'forbidden host'}), 403
    origin = request.headers.get('Origin')
    if origin is not None and origin not in ALLOWED_ORIGINS:
        return jsonify({'status': 'error', 'error': 'forbidden origin'}), 403

session_id = None

# Console messages are pushed to any listening browser tabs over SSE.
message_subscribers = []
message_subscribers_lock = threading.Lock()

def add_to_transcript(text):
    with message_subscribers_lock:
        subscribers = list(message_subscribers)
    for q in subscribers:
        q.put(text)

@app.route('/api/console-stream')
def console_stream():
    def gen():
        q = queue.Queue()
        with message_subscribers_lock:
            message_subscribers.append(q)
        try:
            while True:
                text = q.get()
                # Blank lines/newlines within a message would break the SSE
                # framing, so send each line as its own "data:" field.
                for line in text.splitlines() or ['']:
                    yield f'data: {line}\n'
                yield '\n'
        finally:
            with message_subscribers_lock:
                if q in message_subscribers:
                    message_subscribers.remove(q)
    return Response(gen(), mimetype='text/event-stream')

@app.route('/')
def serve_index():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(this_dir, '..' )
    return send_from_directory(parent_dir, 'speech_listener.html')

@app.route('/ptt_pointer.png')
def serve_pointer():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(this_dir, '..', 'media' )
    return send_from_directory(parent_dir, 'ptt_pointer.png')

@app.route('/settings-778_32.png')
def serve_settings_icon():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(this_dir, '..', 'media' )
    return send_from_directory(parent_dir, 'settings-778_32.png')

@app.route('/listener_closed.html')
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

@app.route('/api/reset-fsm', methods=['POST'])
def handle_reset_fsm():
    global robot
    print('Resetting state machine...')
    add_to_transcript('Resetting state machine...')
    # The state machine belongs to the robot's event loop, so reset it there.
    robot.loop.call_soon_threadsafe(reset_fsm)
    return jsonify({'status': 'ok'})

def reset_fsm():
    # Runs on the robot's event loop.
    global running_fsm
    for child in running_fsm.children.values():
        child.stop()
    if 'reset_fsm' in running_fsm.children:
        running_fsm.children['reset_fsm'].start()
    elif running_fsm.start_node:
        running_fsm.start_node.start()
    else:
        pass
    # Play the sound after stopping, without waiting for it to finish.
    this_dir = os.path.dirname(os.path.abspath(__file__))
    media_path = os.path.abspath(os.path.join(this_dir, '..', 'media', 'reset_fsm.mp3'))
    playsound(media_path, block=False)


@app.route('/api/confirmation-bell', methods=['GET', 'POST'])
def handle_confirmation_bell():
    global speech_listener
    if request.method == 'POST':
        data = request.json
        speech_listener.set_confirmation(bool(data['enabled']))
        return jsonify({'status': 'ok'})
    return jsonify({'enabled': speech_listener.confirmation_bell})


DOCUMENT_MAX_BYTES = 200 * 1024 * 1024
app.config['MAX_CONTENT_LENGTH'] = DOCUMENT_MAX_BYTES
DOCUMENT_TEXT_MAX_BYTES = 200 * 1024  # text documents are re-sent to the LLM every turn

@app.errorhandler(413)
def handle_too_large(exc):
    # Flask rejects bodies over MAX_CONTENT_LENGTH before any route runs;
    # reply in JSON so the page can show a readable message.
    return jsonify({'status': 'error',
                    'error': 'file is larger than %d MB' % (DOCUMENT_MAX_BYTES // (1024 * 1024))}), 413

@app.route('/api/upload-script', methods=['POST'])
def handle_upload_script():
    global robot
    file = request.files.get('file')
    if file is None or file.filename == '':
        return jsonify({'status': 'error', 'error': 'no file'}), 400
    if getattr(robot, 'openai_client', None) is None:
        return jsonify({'status': 'error', 'error': 'no OpenAI client is configured'}), 503

    data = file.read()  # size already capped by MAX_CONTENT_LENGTH
    try:
        document = prepare_document(file.filename, data, DOCUMENT_TEXT_MAX_BYTES)
    except DocumentError as exc:
        return jsonify({'status': 'error', 'error': str(exc)}), 400

    if document.kind == 'pdf':
        # attach_pdf blocks (possibly for minutes) and doesn't touch
        # event-loop state, so it runs here on the Flask thread.
        try:
            robot.openai_client.attach_pdf(document.saved_name, document.data)
        except Exception as exc:
            print("*** OpenAI PDF indexing failed: %s" % exc)
            return jsonify({'status': 'error',
                            'error': 'OpenAI could not accept this PDF: %s' % exc}), 502
        print("Indexed PDF '%s' (%d bytes) for retrieval" %
              (document.saved_name, len(document.data)))
        return jsonify({'status': 'ok',
                        'name': document.saved_name,
                        'sourceBytes': len(data),
                        'kind': document.kind})

    dest = pathlib.Path.home() / 'Documents' / 'Celeste' / document.saved_name
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(document.text, encoding='utf-8')
    context = 'Contents of uploaded document "%s":\n\n%s' % (document.saved_name,
                                                                document.text)
    robot.loop.call_soon_threadsafe(robot.openai_client.note_for_later, context)
    text_bytes = len(document.text.encode('utf-8'))
    print("Loaded text '%s' (%d bytes) into character's context" %
          (document.saved_name, text_bytes))
    return jsonify({'status': 'ok',
                    'name': document.saved_name,
                    'sourceBytes': len(data),
                    'textBytes': text_bytes,
                    'kind': document.kind})


@app.route('/api/speech-to-text', methods=['POST'])
def handle_speech_to_text():
    global speech_listener
    data = request.json
    speech_listener.handle_utterance(data['text'])
    return jsonify({'status': 'ok'})

class SpeechListener():
    UTTERANCE_EXPIRATION_TIME = 10 # seconds

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
        self.utterance_queue = []  # list of entries of form [time, string, words]
        self.utterance_queue_lock = threading.Lock()

    def run_flask(self):
        # Suppress the default Flask logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        # Debug must be false to prevent duplicate tab:
        app.run(port=LISTENER_PORT, debug=False, use_reloader=False, threaded=True)

    def load_listener_page(self):
        webbrowser.open_new_tab('http://127.0.0.1:%d/' % LISTENER_PORT)
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
        if len(string) == 0:
            print("Heard: (nothing)")
            add_to_transcript("Heard: (nothing)")
            return
        if self.confirmation_bell:
            this_dir = os.path.dirname(os.path.abspath(__file__))
            media_path = os.path.abspath(os.path.join(this_dir, '..', 'media', 'acknowledge4.mp3'))
            # A non-blocking playsound() goes silent when the thread that
            # started it exits, and this Flask request thread exits right
            # away.  So play the sound with a blocking call.
            playsound(media_path)
            # Alternativee:  play the sound with blocking on a thread of its own:
            #threading.Thread(target=playsound, args=(media_path,), daemon=True).start()
        self.append_utterance_queue(string, words)
        # We're on a Flask request thread; the event router belongs to the
        # robot's event loop, so do the delivery there.
        self.robot.loop.call_soon_threadsafe(self.deliver_if_listening)

    def deliver_if_listening(self):
        # Runs on the robot's event loop.
        if self.robot.erouter.dispatch_table.get(SpeechEvent):
            self.pop_utterance_queue()

    def append_utterance_queue(self, string, words):
        with self.utterance_queue_lock:
            self.utterance_queue.append([time.time(), string, words])

    def pop_utterance_queue(self):
        # Runs on the robot's event loop, but Flask request threads append to
        # the queue concurrently, so queue access is guarded by a lock.  The
        # lock is released before posting the event so a handler can safely
        # call back into the queue.
        while True:
            with self.utterance_queue_lock:
                if not self.utterance_queue:
                    return
                (creation_time, string, words) = self.utterance_queue.pop(0)
            if time.time() - creation_time <= self.UTTERANCE_EXPIRATION_TIME:
                print("Heard: '%s'" % string)
                sys.stdout.flush()
                add_to_transcript("Heard: '%s'" % string)
                event = SpeechEvent(string, words)
                self.robot.erouter.post(event)
                return
            # entry was stale; try the next one
            print("Discarded (timed out): '%s'" % string, flush=True)
            add_to_transcript("Discarded (timed out): '%s'" % string)

    def start(self):
        if self.robot.flask_thread:
            return
        self.robot.flask_thread = threading.Thread(target=self.run_flask)
        self.robot.flask_thread.start()
        self.robot.loop.call_later(1, self.load_listener_page)
