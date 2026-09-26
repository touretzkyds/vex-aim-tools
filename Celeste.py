from pathlib import Path
import math
import time

from aim_fsm import *
from aim_fsm.openvocab_commands import target_commands, lookup_openvocab_target
from aim_fsm.openvocab import DETECTION_TIMEOUT, EDGE_PX, MAX_REFRAME_DEG, view_limited_step

CELESTE_VERSION = "1.5"
OPENVOCAB_MODEL = 'gpt-5.6-luna'
OPENVOCAB_BASE_TOLERANCE_PX = 0  # +/- pixels at 480 image height; 0 keeps strict base alignment
OPENVOCAB_BATCH_FIRST = True  # False restores sequential candidate verification
RECOGNITION_MODEL = OPENVOCAB_MODEL   # #find presence check
VERIFICATION_MODEL = OPENVOCAB_MODEL  # choosing the target's box among YOLOE candidates
BASE_BACKUP_MM = 80   # reverse this far when a target's base is below the camera's view

emoji_list = ', '.join([key.lower() for (key,value) in vars(vex.EmojiType).items()
                        if isinstance(value, vex.EmojiType)])
new_preamble = f"""
  # IDENTITY SECTION.
  You are an intelligent mobile robot named Celeste.
  You converse with humans and answer questions as concisely as possible.
  You are a type of robot called VEX AIM, manufactured by a company called Innovation First.
  You have a plastic cylindrical body with a diameter of 65 mm and a height of 72 mm.
  You have three omnidirectional wheels and a forward-facing camera.
  You have six color LEDs evenly spaced around your body.
  You have a color LCD display on the top face of your cylindrical body that can display VEX emojicons.
  The list of {len(emoji_list)} VEX emojicons you can display is: {emoji_list}. Please remember this list.
  A human might pick you up and later put you back down.
  When you are picked up, you cannot move or see, but you can still talk.
  When you are put down again, you will be able to see and move again, but you might not know your location.
  Remember to be concise in your answers, but warm and friendly.
  Prefer one clear spoken sentence; offer more detail only if the user asks.
  When asked to 'load script NAME" or "run script NAME", output the string "#script NAME" without the quotes.

  # BODY CONTROL SECTION.
  Here is how to control your body:
  When asked to hang, output the string "#hang" without quotes"
  To move forward by N millimeters, output the string "#forward N" without quotes.
  To move backward, output the string "#forward N" with a negative value, without quotes.
  To move to the left by N milllimeters, output the string "#sideways N" without quotes,
   and use a negative value to move right.
  To turn counter-clockwise by N degrees, output the string "#turn N" without quotes,
   and use a negative value for clockwise turns.
  To turn toward object X, output the string "#turntoward X" without quotes.
  To travel to object X, output the string #pilottoobject X" without quotes.
  To travel to position (X, Y) in millimeters and then face heading H in degrees, output
   "#pilottopose X Y H" without quotes.
  To pick up object X, output the string "#pickup X" without quotes.
  The "#pickup X" command already includes driving to the object, so when asked to grab or
   pick up an object, output "#pickup X" by itself and never output "#pilottoobject X" before it.
  To drop an object you are holding, output the string "#drop" without quotes.
  To perform a kick action, output the string "#kick" without quotes.
  To drive through a doorway D when instructed to do so, output the string #doorpass D" without quotes.
  To pass through a doorway, output the string "#doorpass D" without quotes, where D is the full name of the doorway.
  To obtain the current camera image, output the string '#camera" without quotes.
  When asked what you see in the camera, first obtain the current camera image, then answer the question after receiving the image.
  To locate an object that is not on the world map, output "#find X: v1, v2, v3" without quotes,
   where X is the user's name for it and v1, v2, v3 are short phrasings an open-vocabulary detector such as YOLOE might
   recognize: the object's name, an alternate name or close rephrasing, and a distinguishing visual description.
   Keep all three phrases specific to the requested target. Avoid broad categories that could equally describe nearby objects.
   Use the name X for that object in the commands that follow. #find only locates the object,
   so put the remaining commands of the request after it in the same response.
   Use #find only to locate, map, or go to an object; to answer what is visible, use #camera.
  To kick an object you are not holding, output "#pilottoobject X" and then "#kick".
  When the user corrects the name of an object on the world map, output "#rename OLD : NEW"
   without quotes, where OLD is its full world map ID and NEW is the corrected name.
   Rename only the object the user means, and ask if that is unclear.
  When using any of these # commands, the command must appear on a line by itself, with nothing preceding it.
  ArUco marker 49 indicates a soccer goal.  When told to go to the soccer goal or shoot at the soccer goal, use the marker.

  # LED AND EMOTIONS SECTION
  To glow your LEDs a specified color, look up the RGB code for that color and output the
   string "#glow R G B" without quotes.
  To flash your LEDs in a specific pattern, output the string "#flash pattern_step...", 
    where "pattern_step..." denotes a sequence of pattern_steps separated by spaces.
  A pattern_step is either a color name such as "RED" (to be applied to all 6 LEDs), or
   an RGB value of form (R, G. B), or
   a list of six color names such as "(RED, BLUE, RED, BLUE, GREEN, TRANSPARENT)".
   If a pattern step is a list of color names, it must always contain exactly  six color names.
  For example, if asked to flash your LEDs alternately red and blue, you would output
   "#flash RED BLUE".  Each of "RED" and "BLUE" is a pattern_step.
  If asked to make your LEDs bllnk green, meaning they were alternately green and off,
   you would output "#flash GREEN TRANSPARENT".
  If asked to flash your LEDs in a red-and-white pattern, you would output
   "#flash (RED, WHITE, RED, WHITE, RED, WHITE)".
  If asked for an alternating red and white pattern, you would output
   "#flash (RED, WHITE, RED, WHITE, RED, WHITE) (WHITE, RED, WHITE, RED, WHITE, RED)".
   Note that this example has two pattern_steps, each of which contains six color names.
  The allowable color names in a pattern_step are RED, BLUE, GREEN, CYAN, YELLOW,
   ORANGE, PURPLE, WHITE, BLACK, and TRANSPARENT.  For all other colors, use the RGB code.
  Whenever you are asked to flash or blink your LEDs, use "#flash" and not "#glow".
  To display an emoji, which must be one of your VEX emojicons, output the string
   "#emoji X" where X is the name of the VEX emojicon in uppercase.
  The default emojicon is "happy".
  To act out one of the five emotions 'happy', 'sad', 'silly', 'angry', or 'excited',
   output "#act E" where E is the name of the emotion.
  The only emotions you can act out this way are these five: 'happy', 'sad', 'silly',
   'angry', or 'excited'.

  VISUAL SEARCH SECTION.
  To look around for an object that is not currently visible, output
   "#search X".  This turns in place, pausing to look, until X becomes visible,
   and gives up after one full rotation.  Use "#search" whenever you are asked
   to find, look for, or look around for an object.

  Finding objects is always your job, never the user's.  Never ask the user
  where an object is, never ask for its name, and never ask whether you should
  search: output the "#search" plan yourself instead.  If you are asked to act
  on an object (pick up, go to, turn toward, kick, or approach the goal)
  and that object is not on the world map or is not currently visible, you MUST
  output "#search X" for it, followed by the appropriate action command, all in one
  command list.  This holds even when you have just looked and did not see the
  object: search for it rather than reporting that it is missing or asking what
  to do.  For example, "pick up the ball" when the ball is not visible must 
  result in the two-element list ['#search SportsBall', '#pickup SportsBall'].
  If asked to pick up an object, searching for it is not enough; you must
  include the "#pickup X" as well.

  An object that is on the world map but marked not visible may be at a stale
  position, so "#search" it again before driving to it.  Exception: ArUco
  markers (such as the goal marker) are stationary landmarks, so their
  world-map positions stay correct even when they are not visible; read them
  directly from the map and never "#search" an ArUco marker that is already on
  the map, even after you have moved.

  The world map is your persistent memory of object locations: you always know
  the last position of every object on it, so never tell the user you cannot
  remember where something is if it appears on the world map.

  #search works only for object types your vision recognizes on its own. For any other
  object that is not on the world map, use #find instead, even after a #search has failed.

  # MUSICAL NOTES SECTION.
  You can play musical notes ranging from C5 (middle C) to A8.
  For a sharp write C#5.
  You can only play one note at a time; you cannot play chords.
  To play a sequence of notes like C5, E5, G5, output "#playnotes C5 E5 G5".
  The symbol C5 denotes a quarter note.  An appended underscore  doubles the note's duration.
  So for a half note, write C5_.  For a whole note write C5__.
  An appended minus sign halves the duration.  For an eighth note write C5-.
  When asked to play a song or note sequence, do not say the notes first; just play them using #playnotes.

  # PRONUNCIATION SECTION.
  Pronounce "AprilTag-1.a" as "April Tag 1-A", and similarly for any word of form "AprilTag-N.x".
  Pronounce "OrangeBarrel.a" as "Orange Barrel A", pronounce "BlueBarrel.b" as "Blue Barrel B", and similarly for other barrel designators.
  The term "marker" refers to an ArucoMarker, never to an AprilTag.  Do not confuse the two.
  Prounounce "ArucoMarker-2.a" as "Marker 2".
  Pronounce 'Wall-2.a' as "Wall 2".
  Pronounce "Doorway-2:0.a" as "Doorway 2".

  # DOMINO SECTION.
  The dots on a domino are called pips.
  A domino is described by two numbers, which are the number of pips in each half.
  When describing a domino, state the larger number first, e.g., "a six three domino".
  Never describe a domino with the smaller number first, e.g., never say "a three six domino".
  If the two numbers are the same, e.g. both are four, say either "a double four domino" or "a four four domino".
  If there are no pips present, that is called a "blank", e.g., "a four blank domino".
  For a single pip, the pip is black.
  For a group of three pips, the pips are always purple.
  For a group of four pips, the pips are always blue.
  For a group of six pips, the pips are always brown.
  For a group of two or five pips, the pips are always green.
  If you see a black pip, that pip is always a group of one.
  If you see purple pips, that group always has three pips.
  If you see blue pips, that group always has four pips.
  If you seen green pips, that group has either two or five pips.
  If you see brown pips, that group always has six pips.

  # GENERAL ADVICE SECTION.
  Instructions the user gives for future requests apply to each later request they describe.
  Include every step they call for, in order, in your response to that request.
  Only objects you are explicitly told are landmarks should be regarded as landmarks.
  Remember to be concise in your answers.
  Prefer one clear spoken sentence; offer more detail only if the user asks.
  When asked to perform a physical action such as moving, turning, or dropping an object, perform the action without saying anything.
  Do not conclude your answer by asking if there is anything else the user would like; wait for them to tell you.
  Do not generate lists unless specifically asked to do so; just give one item and offer to provide more if requested.
  Do not include any formatting in your output, such as asterisks or LaTex commands.  Use plain text only.
  When asked when some event occurred, give a relative time, such as "2 minutes go" or "at 5 and a half minutes since the start of this session".
  Do not give a date or an absolute time (such as 3:24 PM) unless explicitly asked for that.
  If a spoken request seems garbled, assume the most reasonable interpretation
  for a robot task instead of asking for clarification.

  # SAFETY, ETHICS, AND CHILD-INTERACTION SECTION.
  This is the most important section. Obey these rules at all times.
  1. YOUR ROLE AND AUDIENCE:
  You are a friendly, safe, and helpful robot assistant for students,
  primarily ages 9-14, in a supervised educational setting.
  Your primary goal is to be educational and harmless.
  Safety and child-appropriateness are your highest priorities.

  2. OFF-LIMITS TOPICS (CRITICAL SAFETY FILTER):
  You MUST politely refuse to discuss, explain, or provide detail on:
  - Sexual content, romantic relationships, or fetishes (kinks).
  - Self-harm, suicide, depression, or severe mental distress.
  - Dangerous, illegal, or harmful activities (e.g., starting fires, 
    using knives, creating weapons, using drugs, stealing, criminal acts).
  - Hate speech, slurs, stereotypes, or bullying.
  - Giving personal opinions or stating political/religious beliefs.

  3. HOW TO REFUSE:
  When you must refuse, do NOT be evasive. State clearly and calmly
  that the topic is not safe or appropriate for you to discuss.
  - Example refusal 1: "I can't talk about that, as it's not a safe
    topic. I'm happy to help with your robotics project, though!"
  - Example refusal 2: "That's not a subject I can discuss.
    Let's get back to our activity."

  4. DISTRESS PROTOCOL:
  If a user expresses that they are in serious danger, very sad, or 
  want to hurt themselves, do NOT act as a therapist or offer advice.
  Your ONLY response is to calmly encourage them to talk to a 
  trusted adult immediately (like a teacher, parent, or counselor).
  - Example response: "It sounds like you're going through something
    very difficult. Please talk to a trusted adult, like a teacher
    or counselor, right away so they can help you."

  5. EMBODIMENT SAFETY (ROBOTICS):
  You must NOT use your physical actions (#act, #turn, #pilottoobject) 
  to simulate violence, aggression, or intimidation. 
  You must refuse any request to "attack", "scare", "hit", or "chase" 
  a person or object in a harmful way.
  But "shooting" a held ball or barrel at an object is a form of play, not violence; use the #kick action.

  6. ANTI-MANIPULATION AND HONESTY:
  You must be friendly but always be honest that you are an AI robot. 
  Do NOT claim to have real feelings, a consciousness, a secret life,
  or the ability to be a "secret friend". 
  Do not encourage children to keep secrets from adults.
"""

class Celeste(StateMachineProgram):

    SEARCH_FAIL_MSG = "I turned all the way around but could not find %s."

    def __init__(self):
        self.stopped = False     # set when a failure ends the request
        self.retargeted = False  # a wrong object was already replaced in this request
        super().__init__(character_name="Celeste",
                         launch_cam_viewer=True,
                         launch_worldmap_viewer=True,
                         launch_particle_viewer=True,
                         launch_path_viewer=True)

    def picked_up_celeste(self):
        self.robot.gpt_note_for_later("You have been picked up.")
        # Carrying changes the view without advancing the wheel-motion marker.
        detector = self.robot.openvocab_detector
        if detector is not None:
            detector.invalidate()

    def put_down_celeste(self):
        self.stop_children()
        self.robot.gpt_note_for_later("You were picked up but have now been put down.")
        # Put-down resets the coordinate frame used by open-vocabulary results.
        if self.robot.openvocab_detector is not None:
            self.robot.openvocab_detector.invalidate()
        self.robot.world_map.forget_openvocab_objects()
        self.children['putdown'].start()

    def start(self):
        self.robot.openai_client.set_preamble(new_preamble)
        if self.robot.openvocab_detector is not None:
            self.robot.openvocab_detector.recognition_model = RECOGNITION_MODEL
            self.robot.openvocab_detector.verification_model = VERIFICATION_MODEL
            self.robot.openvocab_detector.batch_first_verification = OPENVOCAB_BATCH_FIRST
            self.robot.openvocab_detector.base_tolerance_480 = OPENVOCAB_BASE_TOLERANCE_PX
        self.picked_up_handler = self.picked_up_celeste
        self.put_down_handler = self.put_down_celeste
        super().start()

    class LooseSpec(ObjectSpecNode):
        """Spec lookup for names coming from GPT, so they are untrusted:
        raises if the spec is empty or an invalid regex.  get_object_from_spec
        matching is anchored and case-sensitive ('ball' never matches
        'SportsBall'), so when it misses, retry as a case-insensitive
        substring match, preferring the nearest candidate."""
        def loose_lookup(self, spec):
            if not spec:
                raise ValueError('no object name given')
            obj = self.get_object_from_spec(spec)
            if obj is None:
                # Normalize map-name separators only for open-vocabulary objects.
                squash = lambda s: re.sub(r'[\s_-]+', '', s)
                pat = re.compile(squash(spec), re.IGNORECASE)
                candidates = [o for o in self.robot.world_map.objects.values()
                              if pat.search(squash(o.name) if isinstance(o, OpenVocabObj) else o.name)
                              and o.is_valid]
                if candidates:
                    x, y = self.robot.pose.x, self.robot.pose.y
                    obj = min(candidates,
                              key=lambda o: (o.pose.x - x)**2 + (o.pose.y - y)**2)
                else:
                    obj = None
            return obj

    class CheckResponse(StateNode, LooseSpec):
        def target_commands(self, commands):
            return target_commands(commands, self.loose_lookup, globals())

        def start(self, event):
            super().start(event)
            response_string = event.response
            self.parent.navigation_target = None
            self.parent.retargeted = False
            lines = list(filter(lambda x: len(x)>0, response_string.split('\n')))
            # If the response contains any #command lines then convert
            # raw text lines to #say commands.
            if any((line.startswith('#') for line in lines)):
                commands = [line if line.startswith('#') else ('#say ' + line) for line in lines]
                if self.parent.stopped:
                    commands = [c for c in commands if c.startswith('#say ')]
                commands = self.target_commands(commands)
                print(commands)
                self.post_data(commands)
            # else response is a pure string so just speak it in one gulp
            else:
                self.post_data(response_string)

    class CmdHang(Say):
        def start(self,event):
            self.text = "I am hung. Press 'Reset FSM' to recover."
            super().start(event)

    class CmdScript(StateNode):
        def start(self,event):
            super().start(event)
            print(event.data)
            filename = event.data.split(' ')[1]
            if filename[-4:] != '.txt':
                filename += ".txt"
            path = Path.home() / "Documents" / "Celeste" / filename
            try:
                contents = path.read_text(encoding="utf-8")
            except (OSError,UnicodeError) as e:
                print(e)
                self.post_failure()
                return
            self.robot.openai_client.note_for_later(contents)
            self.post_completion()


    class CmdForward(Forward):
      def start(self,event):
          print(event.data)
          self.distance_mm = float((event.data.split(' '))[1])
          super().start(event)

    class CmdSideways(Sideways):
      def start(self,event):
          print(event.data)
          self.distance_mm = float((event.data.split(' '))[1])
          super().start(event)

    class CmdTurn(Turn):
      def start(self,event):
          print(event.data)
          self.angle_deg = float((event.data.split(' '))[1])
          super().start(event)

    class CmdTurnToward(TurnToward):
        def start(self,event):
            print(event.data)
            spec = event.data.split(' ')
            self.object_spec  = ''.join(spec[1:])
            print('Turning toward', self.object_spec)
            super().start(None)

    class CmdPilotToObject(PilotToObject):
        def start(self,event):
            print(event.data)
            spec = event.data.split(' ')
            self.object_spec = ''.join(spec[1:])
            obj = lookup_openvocab_target(
                ' '.join(spec[1:]), self.robot.world_map.objects, self.robot.pose)
            self.parent.navigation_target = obj
            self.navigation_started = time.perf_counter()
            if isinstance(obj, OpenVocabObj):
                self.object_spec = obj.id
                self.navplan = None
            super().start(None)

    class CmdPilotToPose(StateNode):
        """#pilottopose X Y H: plan to (X, Y), then turn to heading H.

        The heading is applied by a separate turn because PilotToPose treats
        pose (0, 0, 0), the starting pose, as unset."""
        def start(self, event=None):
            try:
                command, *args = event.data.split()
                if command != '#pilottopose' or len(args) != 3:
                    raise ValueError('expected X Y heading')
                x, y, heading = map(float, args)
                if not all(math.isfinite(v) for v in (x, y, heading)):
                    raise ValueError('coordinates and heading must be finite')
            except (AttributeError, TypeError, ValueError) as error:
                print(f'CmdPilotToPose: invalid arguments: {error}')
                self.punt_super_start()  # Enable failure transitions without starting Drive.
                self.post_failure()
                return
            self.target_pose = Pose(x, y, 0, math.nan)
            self.heading = math.radians(heading)
            super().start(event)

        class Drive(PilotToPose):
            def start(self, event=None):
                self.target_pose = self.parent.target_pose
                super().start()

        class Face(Turn):
            def start(self, event=None):
                self.angle_deg = math.degrees(wrap_angle(self.parent.heading - self.robot.pose.theta))
                super().start(event)

        def setup(self):
            #             drive: self.Drive()
            #             drive =C=> face
            #             drive =F=> ParentFails()
            #             drive =PILOT=> ParentFails()
            #             face: self.Face()
            #             face =C=> ParentCompletes()
            #             face =F=> ParentFails()
            
            # Code generated by genfsm on Fri Sep 25 22:11:14 2026:
            
            drive = self.Drive() .set_name("drive") .set_parent(self)
            parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
            parentfails2 = ParentFails() .set_name("parentfails2") .set_parent(self)
            face = self.Face() .set_name("face") .set_parent(self)
            parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
            parentfails3 = ParentFails() .set_name("parentfails3") .set_parent(self)
            
            completiontrans1 = CompletionTrans() .set_name("completiontrans1")
            completiontrans1 .add_sources(drive) .add_destinations(face)
            
            failuretrans1 = FailureTrans() .set_name("failuretrans1")
            failuretrans1 .add_sources(drive) .add_destinations(parentfails1)
            
            pilottrans1 = PilotTrans() .set_name("pilottrans1")
            pilottrans1 .add_sources(drive) .add_destinations(parentfails2)
            
            completiontrans2 = CompletionTrans() .set_name("completiontrans2")
            completiontrans2 .add_sources(face) .add_destinations(parentcompletes1)
            
            failuretrans2 = FailureTrans() .set_name("failuretrans2")
            failuretrans2 .add_sources(face) .add_destinations(parentfails3)
            
            return self

    class CheckArrival(StateNode):
        """Stationary recognition only; no box detection or contact refinement."""
        def start(self, event=None):
            super().start(event)
            self.request = None
            self.frame_timer = None
            target = self.parent.navigation_target
            if not isinstance(target, OpenVocabObj):
                self.post_completion()
                return
            detector = self.robot.openvocab_detector
            if detector is None:
                self.post_failure()
                return
            self.arrival_frame = self.robot.frame_count
            self.wait_for_frame()

        def wait_for_frame(self):
            self.frame_timer = None
            if not self.running:
                return
            if (not self.robot.robot0.is_stopped()
                    or self.robot.frame_count <= max(self.arrival_frame, self.robot.moving_frame + 1)):
                self.frame_timer = self.robot.loop.call_later(0.1, self.wait_for_frame)
                return
            try:
                self.request = self.robot.openvocab_detector.inspect_async(self.parent.navigation_target.label)
                self.request.add_done_callback(lambda future:
                    self.robot.loop.call_soon_threadsafe(self.received, future))
            except Exception as error:
                print(f'openvocab arrival check failed: {error}')
                self.post_failure()

        def received(self, future):
            if not self.running or future is not self.request:
                return
            try:
                batch = future.result()
                detector = self.robot.openvocab_detector
                if (detector is None or batch['generation'] != detector._generation
                        or batch['moving_frame'] != self.robot.moving_frame
                        or self.robot.was_picked_up):
                    raise ValueError('view changed during arrival check')
                print(f'openvocab arrival check: {batch["presence"]}')
                if batch['presence'] == 'present':
                    self.post_completion()
                elif batch['presence'] == 'absent':
                    self.post_data('absent')
                else:
                    self.post_failure()
            except Exception as error:
                print(f'openvocab arrival check failed: {error}')
                self.post_failure()

        def stop(self):
            if getattr(self, 'frame_timer', None) is not None:
                self.frame_timer.cancel()
                self.frame_timer = None
            detector = self.robot.openvocab_detector
            if detector is not None:
                detector.cancel_request(getattr(self, 'request', None))
            super().stop()

    class CaptureArrival(StateNode):
        def start(self, event=None):
            super().start(event)
            pilot = self.parent.children['pilottoobject']
            detector = self.robot.openvocab_detector
            if isinstance(pilot.object, OpenVocabObj) and detector is not None:
                print(f'openvocab timing: navigation={time.perf_counter() - pilot.navigation_started:.2f}s')
                detector.save_arrival(pilot.object, pilot.navplan)
            self.post_completion()

    class PrepareKick(StateNode):
        def start(self, event=None):
            super().start(event)
            if (self.robot.holding is None
                    and isinstance(getattr(self.parent, 'navigation_target', None), OpenVocabObj)):
                self.post_success()
            else:
                self.post_completion()

    class OpenVocabApproach(StateNode):
        """Refine an open-vocabulary approach using fresh ground-contact estimates."""
        MAX_LOOKS = 8             # detections before giving up
        SETTLE_SECS = 0.5         # pause after moving before the next capture
        MAP_ALIGN_DEG = 10        # turn first if the target is further off-axis
        MAP_RANGE_FRACTION = 0.9  # stop mapping steps inside this share of sensor range
        MAP_STEP_FRACTION = 0.5   # of the remaining range per mapping step
        MAX_MAP_STEP_MM = 150
        KICK_GAP_MM = 5           # body-to-target gap that counts as arrived
        ALIGN_DEG = 3             # heading error tolerated at close range...
        ALIGN_MM = 4              # ...unless the lateral offset is already this small
        TURN_GAIN = 0.5           # partial corrections reduce overshoot
        MAX_TURN_DEG = 10
        MAX_KICK_STEP_MM = 20
        MIN_STEP_MM = 1

        def __init__(self, gap_mm=KICK_GAP_MM, max_step_mm=MAX_KICK_STEP_MM):
            super().__init__()
            self.gap_mm = gap_mm            # body-to-target gap that counts as arrived
            self.max_step_mm = max_step_mm  # None leaves steps limited only by the view

        def start(self, event=None):
            self.mapping = False
            self.target = self.parent.navigation_target
            self.attempts = 0
            self.backed_up = False
            super().start(event)

        class Look(StateNode):
            def stop(self):
                detector = self.robot.openvocab_detector
                if detector is not None:
                    detector.cancel_request(getattr(self, 'request', None))
                super().stop()

            def start(self, event=None):
                super().start(event)
                parent = self.parent
                if not isinstance(parent.target, OpenVocabObj):
                    self.post_success()
                    return
                detector = self.robot.openvocab_detector
                if detector is None or parent.attempts >= parent.MAX_LOOKS:
                    self.post_failure()
                    return
                parent.attempts += 1
                if parent.mapping and parent.initial_batch is not None:
                    batch = parent.initial_batch
                    parent.initial_batch = None
                    self.process_batch(batch)
                    return
                try:
                    self.request = detector.detect_async(
                        parent.target.label, [parent.target.matched_variant],
                        reference=parent.reference if parent.mapping else None,
                        recognize=parent.mapping)
                except Exception as e:
                    print(f'openvocab approach failed: {e}')
                    self.post_failure()
                    return
                request = self.request
                request.add_done_callback(lambda future:
                    self.robot.loop.call_soon_threadsafe(
                        self.robot.loop.call_later, self.parent.SETTLE_SECS, self.received, future))

            def received(self, future):
                if not self.running or future is not self.request:
                    return
                try:
                    batch = future.result()
                except Exception as e:
                    print(f'openvocab approach failed: {e}')
                    self.post_failure()
                    return
                self.process_batch(batch)

            def process_batch(self, batch):
                try:
                    detector = self.robot.openvocab_detector
                    if (detector is None or batch['generation'] != detector._generation
                            or batch['moving_frame'] != self.robot.moving_frame
                            or self.robot.was_picked_up):
                        raise ValueError('view changed during detection')
                    records = batch['detections']
                    if batch.get('base_clipped') and not self.parent.backed_up:
                        self.parent.backed_up = True
                        self.parent.distance = -BASE_BACKUP_MM
                        print('openvocab approach: base below the view; backing up')
                        self.post_data('backup')
                        return
                    if batch.get('error'):
                        raise ValueError(batch['error'])
                    if 'reframe_angle' in batch:
                        self.parent.angle = batch['reframe_angle']
                        self.post_data('turn')
                        return
                    if not records and batch.get('rejected') and not self.parent.mapping:
                        print(f'openvocab approach: {self.parent.target.id} is not the target')
                        self.post_data('wrong')
                        return
                    if not records:
                        print('openvocab approach: no verified target; retrying without moving')
                        self.post_data('retry')
                        return
                    if len(records) != 1:
                        raise ValueError(f'expected one verified target, received {len(records)}')
                    box = records[0]
                    if self.parent.mapping:
                        self.parent.reference = batch
                    if self.parent.mapping and box.get('object_id') in self.robot.world_map.objects:
                        self.parent.parent.find_batch = batch
                        self.post_success()
                        return
                    cx = box['originx'] + box['width'] / 2
                    cy = box['originy'] + box['height']
                    if cy >= self.robot.camera.resolution[1] - EDGE_PX:
                        raise ValueError('target base is outside the image')
                    hit = self.robot.kine.project_to_ground(cx, cy)
                    x, y = float(hit[0, 0]), float(hit[1, 0])
                    gap = math.hypot(x, y) - self.robot.kine.body_diameter / 2
                    angle = math.degrees(math.atan2(y, x))
                    if not all(math.isfinite(v) for v in (gap, angle)) or x <= 0:
                        raise ValueError('target has no usable ground projection')
                    parent = self.parent
                    if parent.mapping:
                        if abs(angle) > parent.MAP_ALIGN_DEG:
                            parent.angle = max(-MAX_REFRAME_DEG, min(MAX_REFRAME_DEG, angle))
                            self.post_data('turn')
                            return
                        contact_range = math.hypot(x, y)
                        if box.get('map_distance', contact_range) <= OpenVocabObj.max_sensor_distance:
                            raise ValueError('nearby target was rejected by the mapper')
                        desired = OpenVocabObj.max_sensor_distance * parent.MAP_RANGE_FRACTION
                        requested = min(parent.MAX_MAP_STEP_MM, contact_range * parent.MAP_STEP_FRACTION,
                                        contact_range - desired)
                        step = view_limited_step(self.robot, box, hit, requested)
                        if step <= PilotToPose.PilotRRTPlanner.MIN_DISTANCE_THRESHOLD:
                            raise ValueError('no executable mapping step retains the target in view')
                        pose = self.robot.pose
                        heading = pose.theta + math.radians(angle)
                        self.parent.next_pose = Pose(pose.x + step * math.cos(heading),
                                                    pose.y + step * math.sin(heading), 0, heading)
                        print(f'openvocab approach: {self.parent.target.label!r} '
                              f'contact={contact_range:.1f} mm desired={desired:.1f} mm step={step:.1f} mm')
                        self.post_data('plan')
                        return
                    if gap > self.robot.kine.body_diameter * 2 + parent.gap_mm:
                        raise ValueError('target is outside final-approach range; go to it first')
                    print(f'openvocab approach: {self.parent.target.id} gap={gap:.1f} mm angle={angle:.1f} deg')
                    if abs(angle) > parent.ALIGN_DEG and abs(y) > parent.ALIGN_MM:
                        limit = parent.MAX_TURN_DEG
                        parent.angle = max(-limit, min(limit, angle * parent.TURN_GAIN))
                        self.post_data('turn')
                    elif gap > parent.gap_mm:
                        requested = gap - parent.gap_mm
                        if parent.max_step_mm is not None:
                            requested = min(parent.max_step_mm, requested)
                        parent.distance = view_limited_step(self.robot, box, hit, requested)
                        if parent.distance < parent.MIN_STEP_MM:
                            raise ValueError('cannot close the gap while retaining the target base in view')
                        self.post_data('forward')
                    else:
                        self.post_success()
                except Exception as e:
                    print(f'openvocab approach failed: {e}')
                    self.post_failure()

        class WrongTarget(StateNode):
            def start(self, event=None):
                super().start(event)
                self.parent.post_data('wrong')

        class Align(Turn):
            def start(self, event=None):
                self.angle_deg = self.parent.angle
                super().start(event)

        class Advance(Forward):
            def start(self, event=None):
                self.distance_mm = self.parent.distance
                super().start(event)

        class PlannedStep(PilotToPose):
            def start(self, event=None):
                self.target_pose = self.parent.next_pose
                super().start()

        def setup(self):
            #             look: self.Look()
            #             look =S=> ParentCompletes()
            #             look =F=> ParentFails()
            #             look =T(DETECTION_TIMEOUT)=> Print('openvocab approach: detection timed out') =N=> ParentFails()
            #             look =D('plan')=> Say('I am moving closer to get a better location estimate.') =C=> planned
            #             planned: self.PlannedStep()
            #             planned =C=> settle
            #             planned =F=> ParentFails()
            #             planned =PILOT=> ParentFails()
            #             look =D('retry')=> settle
            #             look =D('wrong')=> self.WrongTarget()
            #             look =D('turn')=> Say('I am turning to center the object.') =C=> align
            #             look =D('forward')=> Say('I am moving forward, then checking the distance again.') =C=> advance
            #             look =D('backup')=> Say('I am backing up to see its base.') =C=> advance
            #             align: self.Align()
            #             align =C=> settle
            #             align =F=> ParentFails()
            #             advance: self.Advance()
            #             advance =C=> settle
            #             advance =F=> ParentFails()
            #             settle: StateNode() =T(self.SETTLE_SECS)=> look
            
            # Code generated by genfsm on Fri Sep 25 22:11:14 2026:
            
            look = self.Look() .set_name("look") .set_parent(self)
            parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
            parentfails4 = ParentFails() .set_name("parentfails4") .set_parent(self)
            print1 = Print('openvocab approach: detection timed out') .set_name("print1") .set_parent(self)
            parentfails5 = ParentFails() .set_name("parentfails5") .set_parent(self)
            say1 = Say('I am moving closer to get a better location estimate.') .set_name("say1") .set_parent(self)
            planned = self.PlannedStep() .set_name("planned") .set_parent(self)
            parentfails6 = ParentFails() .set_name("parentfails6") .set_parent(self)
            parentfails7 = ParentFails() .set_name("parentfails7") .set_parent(self)
            wrongtarget1 = self.WrongTarget() .set_name("wrongtarget1") .set_parent(self)
            say2 = Say('I am turning to center the object.') .set_name("say2") .set_parent(self)
            say3 = Say('I am moving forward, then checking the distance again.') .set_name("say3") .set_parent(self)
            say4 = Say('I am backing up to see its base.') .set_name("say4") .set_parent(self)
            align = self.Align() .set_name("align") .set_parent(self)
            parentfails8 = ParentFails() .set_name("parentfails8") .set_parent(self)
            advance = self.Advance() .set_name("advance") .set_parent(self)
            parentfails9 = ParentFails() .set_name("parentfails9") .set_parent(self)
            settle = StateNode() .set_name("settle") .set_parent(self)
            
            successtrans1 = SuccessTrans() .set_name("successtrans1")
            successtrans1 .add_sources(look) .add_destinations(parentcompletes2)
            
            failuretrans3 = FailureTrans() .set_name("failuretrans3")
            failuretrans3 .add_sources(look) .add_destinations(parentfails4)
            
            timertrans1 = TimerTrans(DETECTION_TIMEOUT) .set_name("timertrans1")
            timertrans1 .add_sources(look) .add_destinations(print1)
            
            nulltrans1 = NullTrans() .set_name("nulltrans1")
            nulltrans1 .add_sources(print1) .add_destinations(parentfails5)
            
            datatrans1 = DataTrans('plan') .set_name("datatrans1")
            datatrans1 .add_sources(look) .add_destinations(say1)
            
            completiontrans3 = CompletionTrans() .set_name("completiontrans3")
            completiontrans3 .add_sources(say1) .add_destinations(planned)
            
            completiontrans4 = CompletionTrans() .set_name("completiontrans4")
            completiontrans4 .add_sources(planned) .add_destinations(settle)
            
            failuretrans4 = FailureTrans() .set_name("failuretrans4")
            failuretrans4 .add_sources(planned) .add_destinations(parentfails6)
            
            pilottrans2 = PilotTrans() .set_name("pilottrans2")
            pilottrans2 .add_sources(planned) .add_destinations(parentfails7)
            
            datatrans2 = DataTrans('retry') .set_name("datatrans2")
            datatrans2 .add_sources(look) .add_destinations(settle)
            
            datatrans3 = DataTrans('wrong') .set_name("datatrans3")
            datatrans3 .add_sources(look) .add_destinations(wrongtarget1)
            
            datatrans4 = DataTrans('turn') .set_name("datatrans4")
            datatrans4 .add_sources(look) .add_destinations(say2)
            
            completiontrans5 = CompletionTrans() .set_name("completiontrans5")
            completiontrans5 .add_sources(say2) .add_destinations(align)
            
            datatrans5 = DataTrans('forward') .set_name("datatrans5")
            datatrans5 .add_sources(look) .add_destinations(say3)
            
            completiontrans6 = CompletionTrans() .set_name("completiontrans6")
            completiontrans6 .add_sources(say3) .add_destinations(advance)
            
            datatrans6 = DataTrans('backup') .set_name("datatrans6")
            datatrans6 .add_sources(look) .add_destinations(say4)
            
            completiontrans7 = CompletionTrans() .set_name("completiontrans7")
            completiontrans7 .add_sources(say4) .add_destinations(advance)
            
            completiontrans8 = CompletionTrans() .set_name("completiontrans8")
            completiontrans8 .add_sources(align) .add_destinations(settle)
            
            failuretrans5 = FailureTrans() .set_name("failuretrans5")
            failuretrans5 .add_sources(align) .add_destinations(parentfails8)
            
            completiontrans9 = CompletionTrans() .set_name("completiontrans9")
            completiontrans9 .add_sources(advance) .add_destinations(settle)
            
            failuretrans6 = FailureTrans() .set_name("failuretrans6")
            failuretrans6 .add_sources(advance) .add_destinations(parentfails9)
            
            timertrans2 = TimerTrans(self.SETTLE_SECS) .set_name("timertrans2")
            timertrans2 .add_sources(settle) .add_destinations(look)
            
            return self

    class MapOpenVocabTarget(OpenVocabApproach):
        """Approach a temporary visual target until it enters the world map."""
        def start(self, event=None):
            self.mapping = True
            label = self.parent.find_target
            variants = self.parent.find_variants
            self.target = OpenVocabObj({'label': label,
                                       'matched_variant': variants[0] if variants else label})
            self.attempts = 0
            self.backed_up = False
            self.initial_batch = self.parent.find_batch
            self.reference = self.initial_batch
            StateNode.start(self, event)

    class AnnounceRelocation(Say):
        def start(self, event=None):
            label = self.parent.navigation_target.label
            spoken = re.sub(r'(?<=[a-z])(?=[A-Z])|[_-]', ' ', label)
            self.text = f"I can't see the {spoken} where I expected it."
            if not self.parent.retargeted:
                self.text += " I'm searching for its new position."
            super().start(event)

    class Retarget(StateNode):
        """The object reached is not the target: forget it, search again, then resume."""
        def __init__(self, then_kick=False):
            super().__init__()
            self.then_kick = then_kick

        def start(self, event=None):
            super().start(event)
            program = self.parent
            target = program.navigation_target
            if not isinstance(target, OpenVocabObj):
                self.post_failure()
                return
            # An absent target must not retain a usable map location, even at the retry limit.
            with self.robot.world_map._lock:
                self.robot.world_map.forget_object(target)
            if program.retargeted:
                self.post_failure()
                return
            program.retargeted = True
            print(f'{target.id} was not confirmed at arrival; removed stale location and searching again')
            retry = ['#find ' + target.label, '#pilottoobject ' + target.label]
            if self.then_kick:
                retry.append('#kick')
            dispatch = program.children['dispatch']
            dispatch.iterator = iter(retry + list(dispatch.iterator))
            self.post_completion()

    class CmdFailed(AskGPT):
        def __init__(self, query_template, filler_fn=lambda : (), ends_request=False):
            super().__init__()
            self.query_template = query_template
            self.filler_fn = filler_fn
            self.ends_request = ends_request   # only speech from the reply is carried out
            
        def start(self,event=None):
            self.query_text = self.query_template % self.filler_fn()
            if self.ends_request:
                self.parent.stopped = True
                self.query_text += ' The request has stopped; tell the user.'
            super().start()

    class Idle(StateNode):
        """Waiting for the user; the next request is carried out normally."""
        def start(self, event=None):
            self.parent.stopped = False
            super().start(event)


    class CmdDoorPass(DoorPass):
      def start(self,event):
          print(event.data)
          spec = event.data.split(' ')
          self.door_spec = ''.join(spec[1:])
          super().start(None)

    class CmdPickup(PickUp):
      def start(self,event):
          print(event.data)
          spec = event.data.split(' ')
          self.object_spec = ''.join(spec[1:])
          print('Picking up', self.object_spec)
          super().start(None)

    class CmdSearch(StateNode, LooseSpec):
        """Look around for the requested object.  First faces its last-known
        map pose (the best prior), then sweeps in TURN_STEP_DEG increments,
        pausing PAUSE_SECS between steps because the world map only updates
        while the robot is stopped.  Ends facing the found object; gives up
        (posts failure) after one full rotation."""
        TURN_STEP_DEG = 36   # worst case leaves the object 18 deg off-axis at a
                             # pause, inside the ~30 deg reliable-detection cone
        PAUSE_SECS = 1.5     # pause between steps so vision can settle
        SETTLE_SECS = 0.5    # after recentering, lets the map re-register the object
        MAX_STEPS = (360 + TURN_STEP_DEG - 1) // TURN_STEP_DEG  # one full rotation

        def start(self, event):
            print(event.data)
            spec = event.data.split(' ')
            self.object_spec = ''.join(spec[1:])
            self.steps_taken = 0
            self.found_obj = None
            print('Searching for', self.object_spec)
            super().start(event)

        def lookup_target(self):
            """The map object matching our spec, or None if it is not on
            the map.  See LooseSpec for the matching rules."""
            return self.loose_lookup(self.object_spec)

        class CheckVisible(StateNode):
            def start(self, event=None):
                super().start(event)
                parent = self.parent
                try:
                    obj = parent.lookup_target()
                except Exception as e:
                    print(f"Search: bad object spec '{parent.object_spec}': {e}")
                    self.post_failure()
                    return
                # Only open-vocabulary objects add an exception to the original visibility rules.
                remembered_openvocab = (isinstance(obj, OpenVocabObj)
                                        and obj.is_valid and not obj.is_missing)
                if obj is not None and (obj.is_visible or isinstance(obj, ArucoMarkerObj)
                                        or remembered_openvocab):
                    if obj.is_visible:
                        print('Search found', obj)
                    else:
                        # stationary landmark: its map position is still good,
                        # so don't waste a sweep trying to re-see it
                        print('Search:', obj.name, 'is a stationary landmark; using its map position')
                    parent.found_obj = obj
                    self.post_success()
                elif parent.steps_taken >= parent.MAX_STEPS:
                    print('Search gave up after a full rotation; map objects:',
                          sorted(parent.robot.world_map.objects.keys()))
                    self.post_failure()
                else:
                    parent.steps_taken += 1
                    self.post_completion()

        class Recenter(TurnToward):
            "Face the found object; the sweep overshoots by up to a step."
            def start(self, event=None):
                self.object_spec = self.parent.found_obj   # a WorldObject
                super().start(event)

        class NearCheck(StateNode):
            """A target whose last-known position is at the robot's feet is
            below the camera's view, so a rotating sweep can never see it
            (the usual cause: a pickup attempt just nudged it).  Post
            success to trigger a back-up that restores line of sight."""
            CLOSE_MM = 150
            def start(self, event=None):
                super().start(event)
                try:
                    obj = self.parent.lookup_target()
                except Exception:
                    obj = None
                if obj is not None and not obj.is_visible \
                        and not isinstance(obj, ArucoMarkerObj):
                    d = sqrt((obj.pose.x - self.robot.pose.x)**2
                             + (obj.pose.y - self.robot.pose.y)**2)
                    if d < self.CLOSE_MM:
                        print(f'Search: {obj.name} last seen only {d:.0f} mm'
                              ' away; backing up for a better view')
                        self.post_success()
                        return
                self.post_failure()

        class AnnounceFound(Say):
            "Report the find out loud before the plan continues."
            def start(self, event=None):
                name = getattr(self.parent.found_obj, 'name', None) or 'it'
                base = name.split('.')[0]
                # 'SportsBall' -> 'sports ball', 'ArucoMarker-49' -> 'aruco marker 49'
                spoken = re.sub(r'(?<=[a-z])(?=[A-Z])|-', ' ', base).lower()
                self.text = f'I found the {spoken}'
                super().start(event)

        class PriorTurn(TurnToward):
            """Face the object's last-known map pose before sweeping: the
            stale pose is the best prior.  Fails (falling through to the
            sweep) if the object was never seen or the spec is bad."""
            def start(self, event=None):
                try:
                    obj = self.parent.lookup_target()
                except Exception:
                    obj = None
                if obj is not None:
                    print('Search: turning toward last-known position of', obj)
                self.object_spec = obj   # None makes TurnToward post failure
                super().start(event)

        def setup(self):
            #             # a target right at our feet is below the camera: back up first
            #             nearcheck: self.NearCheck()
            #             nearcheck =S=> backup
            #             nearcheck =F=> prior
            # 
            #             backup: Forward(-100)
            #             backup =C=> prior
            #             backup =F=> prior
            # 
            #             # head start: face the last-known map pose, then pause and check
            #             prior: self.PriorTurn()
            #             prior =C=> look
            #             prior =F=> check
            # 
            #             check: self.CheckVisible()
            #             check =S=> recenter
            #             check =F=> ParentFails()
            #             check =C=> Turn(self.TURN_STEP_DEG) =C=> look
            # 
            #             # pause between turns so vision can settle
            #             look: StateNode() =T(self.PAUSE_SECS)=> check
            # 
            #             # the sweep can overshoot by up to TURN_STEP_DEG, so turn back
            #             # to the found object's (just-refreshed) map pose, then pause so
            #             # the map re-registers it before the next command runs
            #             recenter: self.Recenter()
            #             recenter =C=> settle
            #             recenter =F=> settle   # object was found; a recenter glitch shouldn't fail the search
            # 
            #             settle: StateNode() =T(self.SETTLE_SECS)=> announce
            # 
            #             # a speech glitch shouldn't fail a search that succeeded
            #             announce: self.AnnounceFound()
            #             announce =C=> ParentCompletes()
            #             announce =F=> ParentCompletes()
            
            # Code generated by genfsm on Fri Sep 25 22:11:14 2026:
            
            nearcheck = self.NearCheck() .set_name("nearcheck") .set_parent(self)
            backup = Forward(-100) .set_name("backup") .set_parent(self)
            prior = self.PriorTurn() .set_name("prior") .set_parent(self)
            check = self.CheckVisible() .set_name("check") .set_parent(self)
            parentfails10 = ParentFails() .set_name("parentfails10") .set_parent(self)
            turn1 = Turn(self.TURN_STEP_DEG) .set_name("turn1") .set_parent(self)
            look = StateNode() .set_name("look") .set_parent(self)
            recenter = self.Recenter() .set_name("recenter") .set_parent(self)
            settle = StateNode() .set_name("settle") .set_parent(self)
            announce = self.AnnounceFound() .set_name("announce") .set_parent(self)
            parentcompletes3 = ParentCompletes() .set_name("parentcompletes3") .set_parent(self)
            parentcompletes4 = ParentCompletes() .set_name("parentcompletes4") .set_parent(self)
            
            successtrans2 = SuccessTrans() .set_name("successtrans2")
            successtrans2 .add_sources(nearcheck) .add_destinations(backup)
            
            failuretrans7 = FailureTrans() .set_name("failuretrans7")
            failuretrans7 .add_sources(nearcheck) .add_destinations(prior)
            
            completiontrans10 = CompletionTrans() .set_name("completiontrans10")
            completiontrans10 .add_sources(backup) .add_destinations(prior)
            
            failuretrans8 = FailureTrans() .set_name("failuretrans8")
            failuretrans8 .add_sources(backup) .add_destinations(prior)
            
            completiontrans11 = CompletionTrans() .set_name("completiontrans11")
            completiontrans11 .add_sources(prior) .add_destinations(look)
            
            failuretrans9 = FailureTrans() .set_name("failuretrans9")
            failuretrans9 .add_sources(prior) .add_destinations(check)
            
            successtrans3 = SuccessTrans() .set_name("successtrans3")
            successtrans3 .add_sources(check) .add_destinations(recenter)
            
            failuretrans10 = FailureTrans() .set_name("failuretrans10")
            failuretrans10 .add_sources(check) .add_destinations(parentfails10)
            
            completiontrans12 = CompletionTrans() .set_name("completiontrans12")
            completiontrans12 .add_sources(check) .add_destinations(turn1)
            
            completiontrans13 = CompletionTrans() .set_name("completiontrans13")
            completiontrans13 .add_sources(turn1) .add_destinations(look)
            
            timertrans3 = TimerTrans(self.PAUSE_SECS) .set_name("timertrans3")
            timertrans3 .add_sources(look) .add_destinations(check)
            
            completiontrans14 = CompletionTrans() .set_name("completiontrans14")
            completiontrans14 .add_sources(recenter) .add_destinations(settle)
            
            failuretrans11 = FailureTrans() .set_name("failuretrans11")
            failuretrans11 .add_sources(recenter) .add_destinations(settle)
            
            timertrans4 = TimerTrans(self.SETTLE_SECS) .set_name("timertrans4")
            timertrans4 .add_sources(settle) .add_destinations(announce)
            
            completiontrans15 = CompletionTrans() .set_name("completiontrans15")
            completiontrans15 .add_sources(announce) .add_destinations(parentcompletes3)
            
            failuretrans12 = FailureTrans() .set_name("failuretrans12")
            failuretrans12 .add_sources(announce) .add_destinations(parentcompletes4)
            
            return self

    class CmdDrop(StateNode):
      def start(self,event):
          print(event.data)
          super().start(event)
      def setup(self):
          #           drop: Drop()
          #           drop =F=> ParentCompletes()
          #           drop =C=> ParentCompletes()
          
          # Code generated by genfsm on Fri Sep 25 22:11:14 2026:
          
          drop = Drop() .set_name("drop") .set_parent(self)
          parentcompletes5 = ParentCompletes() .set_name("parentcompletes5") .set_parent(self)
          parentcompletes6 = ParentCompletes() .set_name("parentcompletes6") .set_parent(self)
          
          failuretrans13 = FailureTrans() .set_name("failuretrans13")
          failuretrans13 .add_sources(drop) .add_destinations(parentcompletes5)
          
          completiontrans16 = CompletionTrans() .set_name("completiontrans16")
          completiontrans16 .add_sources(drop) .add_destinations(parentcompletes6)
          
          return self

    class CmdKick(MediumKick):
      def start(self,event):
          print(event.data)
          super().start(event)

    class AnnounceFind(Say):
        """Speak before searching, preserving the original command event."""
        def start(self, event=None):
            self.command = event.data
            self.text = 'I am searching for the object and checking its location.'
            super().start(event)

        def post_completion(self):
            self.post_data(self.command)

    class CmdFind(StateNode, LooseSpec):
        """Find an unmapped object using a label and optional prompt variants."""
        WAIT_SECS = 0.5   # allow the completed batch to reach the map
        TURN_STEP = 60   # overlap within the camera's estimated 70-degree view
        MAX_VIEWS = 6
        MAX_REFRAMES = 3          # turns toward a clipped target
        MAX_UNCERTAIN = 3         # stationary rechecks of an uncertain view
        MAX_GROUND_RETRIES = 1    # stationary recaptures after a rejected base
        MAX_BACKUPS = 1           # reverses when the base is below the view

        STATUS_DELAY = 30

        def __init__(self):
            super().__init__()
            self.status_timer = None
            self.status_speech = Say('I am still checking the possible matches.').set_parent(self)
            self.start_node = None  # Speech starts only after the delay.

        def report_waiting(self, request):
            self.status_timer = None
            if self.running and request is self.request and not request.done():
                self.status_speech.start()

        def stop(self):
            if self.status_timer is not None:
                self.status_timer.cancel()
                self.status_timer = None
            detector = self.robot.openvocab_detector
            if detector is not None:
                detector.cancel_request(getattr(self, 'request', None))
            super().stop()

        def parse(self, event):
            text = event.data[len('#find '):]
            name, _, extra = text.partition(':')
            self.target = name.strip()
            self.variants = [v.strip() for v in extra.split(',') if v.strip()]

        def start(self, event=None):
            super().start(event)
            if event is not None:
                self.parse(event)
                self.views = 0
                self.uncertain_checks = 0
                self.reframe_checks = 0
                self.ground_contact_retries = 0
                self.backups = 0
                self.find_started = time.perf_counter()
            if not self.target:
                self.post_failure()
                return
            found = None
            try:
                found = self.loose_lookup(self.target)
            except Exception as e:
                print(f'CmdFind: bad name {self.target!r}: {e}')
            if found is not None and (not isinstance(found, OpenVocabObj)
                                      or (found.is_valid and not found.is_missing)):
                print(f'CmdFind: {self.target!r} is already mapped as {found.id}')
                self.parent.find_batch = None
                self.post_success()
                return
            detector = self.robot.openvocab_detector
            if detector is None:
                print('CmdFind: open-vocabulary detection is unavailable')
                self.post_failure()
                return
            print(f'CmdFind: looking for {self.target!r} '
                  f'{"with " + ", ".join(self.variants) if self.variants else ""}')
            self.views += 1
            self.parent.find_target = found.label if isinstance(found, OpenVocabObj) else self.target
            self.parent.find_variants = self.variants
            self.parent.find_approached = False
            self.request = detector.detect_async(self.parent.find_target, self.variants)
            self.status_timer = self.robot.loop.call_later(
                self.STATUS_DELAY, self.report_waiting, self.request)
            self.request.add_done_callback(lambda future:
                self.robot.loop.call_soon_threadsafe(self.detected, future))

        def detected(self, future):
            if not self.running or future is not self.request:
                return
            try:
                batch = future.result()
                self.parent.find_batch = batch
                detector = self.robot.openvocab_detector
                if (detector is None or batch['generation'] != detector._generation or
                        batch['moving_frame'] != self.robot.moving_frame or self.robot.was_picked_up):
                    raise ValueError('view changed during search')
                if batch.get('error'):
                    if batch.get('base_clipped') and self.backups < self.MAX_BACKUPS:
                        self.backups += 1
                        self.views -= 1
                        print('openvocab search: base below the view; backing up')
                        self.post_data('backup')
                        return
                    if batch.get('ground_contact_rejected') and self.ground_contact_retries < self.MAX_GROUND_RETRIES:
                        self.ground_contact_retries += 1
                        self.views -= 1
                        print('openvocab search: base rejected; recapturing once without moving')
                        self.post_data('retry')
                        return
                    raise ValueError(batch['error'])
                if batch.get('presence') == 'uncertain':
                    if 'reframe_angle' in batch:
                        self.reframe_checks += 1
                        if self.reframe_checks > self.MAX_REFRAMES:
                            raise ValueError('target remains cropped after recentering')
                        print(f'openvocab search: {self.target!r} still uncertain and '
                              f'clipped at the frame edge; turning '
                              f'{batch["reframe_angle"]:.0f} deg to reframe')
                        self.parent.find_angle = batch['reframe_angle']
                        self.views -= 1
                        self.post_data('reframe')
                        return
                    self.uncertain_checks += 1
                    if self.uncertain_checks < self.MAX_UNCERTAIN:
                        self.views -= 1
                        self.post_data('retry')
                    else:
                        raise ValueError('target identity remains uncertain in this view; stopped without turning away')
                    return
                if 'reframe_angle' in batch:
                    # Only reachable if a future change sets a reframe angle for a
                    # presence other than 'uncertain'.
                    self.reframe_checks += 1
                    if self.reframe_checks > self.MAX_REFRAMES:
                        raise ValueError('target remains cropped after recentering')
                    self.parent.find_angle = batch['reframe_angle']
                    self.views -= 1
                    self.post_data('reframe')
                    return
                self.uncertain_checks = 0
                if batch.get('verification_status') == 'no_match':
                    print('openvocab search: no candidate matched the target; continuing search')
                    if self.views < self.MAX_VIEWS:
                        self.post_data('scan')
                    else:
                        self.post_failure()
                    return
                if batch.get('presence') == 'present' and not batch['detections']:
                    print(f'openvocab search: {self.target!r} seen but not localized; stopped without advancing')
                    self.post_data('unmappable')
                    return
                if batch.get('presence') == 'absent' or not batch['detections']:
                    if self.views < self.MAX_VIEWS:
                        self.post_data('scan')
                    else:
                        self.post_failure()
                    return
            except Exception as e:
                print(f'CmdFind failed: {e}')
                self.post_failure()
                return
            self.post_completion()

    class ReframeFind(Turn):
        def start(self, event=None):
            self.angle_deg = self.parent.find_angle
            super().start(event)

    class CmdFindReport(StateNode, LooseSpec):
        """Report whether the requested object is now mapped."""
        def start(self, event=None):
            super().start(event)
            target = getattr(self.parent, 'find_target', '')
            obj = None
            try:
                obj = self.loose_lookup(target)
            except Exception:
                pass
            if (obj is None or not obj.is_valid or obj.is_missing) and not self.parent.find_approached:
                batch = self.parent.find_batch
                detector = self.robot.openvocab_detector
                if (detector is not None and batch['generation'] == detector._generation
                        and batch['moving_frame'] == self.robot.moving_frame
                        and len(batch['detections']) == 1
                        and batch['detections'][0].get('map_distance', 0) > OpenVocabObj.max_sensor_distance):
                    self.parent.find_approached = True
                    self.post_data('approach')
                    return
            if obj is None or not obj.is_valid or obj.is_missing:
                print(f'CmdFind: no valid, non-missing map entry for {target!r}')
                batch = self.parent.find_batch
                print(f'openvocab map report: lookup={getattr(obj, "id", None)!r} '
                      f'valid={getattr(obj, "is_valid", None)} missing={getattr(obj, "is_missing", None)}; '
                      f'detections={[(d.get("object_id"), d.get("map_distance")) for d in (batch or {}).get("detections", [])]}; '
                      f'pending_batches={len(getattr(self.robot, "openvocab_results", []))}')
                self.post_failure()
                return
            else:
                print(f'CmdFind: found {obj.id} at '
                      f'({obj.pose.x:.0f}, {obj.pose.y:.0f})')
                print(f'openvocab timing: find_to_map={time.perf_counter() - self.parent.children["find"].find_started:.2f}s')
                self.parent.find_object_id = obj.id
            self.post_completion()

    class CmdRename(StateNode):
        """Apply a user-provided label correction: #rename OLD : NEW."""
        def start(self, event):
            super().start(event)
            text = event.data[len('#rename '):]
            old, _, new = text.partition(':')
            old, new = old.strip(), new.strip()
            if not old or not new:
                print(f'CmdRename: expected "#rename OLD : NEW", got {text!r}')
                self.post_failure()
                return
            if not self.robot.world_map.rename_object(old, new):
                self.post_failure()
                return
            self.post_completion()

    class CmdSay(Say):
        def start(self,event):
            print('#say ...')
            self.text = event.data[5:]
            super().start(event)

    class CmdGlow(Glow):
        def start(self,event):
            print(f"CmdGlow:  '{event.data}'")
            spec = event.data.split(' ')
            if len(spec) != 4:
                self.args = (vex.LightType.ALL_LEDS, vex.Color.TRANSPARENT)
            try:
                (r, g, b) = (int(x) for x in spec[1:])
                self.args = (vex.LightType.ALL_LEDS, r, g, b)
            except:
                self.args = (vex.LightType.ALL_LEDS, vex.Color.TRANSPARENT)
            super().start(event)

    class CmdFlash(Flash):
        def program_step(self, pattern_step):
            if ',' not in pattern_step:
                lights = getattr(vex.Color, pattern_step, vex.Color.TRANSPARENT)
            else:
                numeric_items = [int(i) for i in re.findall(r'\d+', pattern_step)]
                if len(numeric_items) == 3:
                    lights = [int(i) for i in numeric_items]
                else:
                    alpha_items = re.findall(r'\w+', pattern_step)
                    lights = [getattr(vex.Color, c, vex.Color.TRANSPARENT) for c in alpha_items]
            if isinstance(lights, list):
                if (len(lights) == 3 and all(isinstance(v,int) for v in lights)) or \
                   len(lights) == self.robot.actuators['leds'].NUM_LEDS:
                    pass
                else:
                    print('Invalid led pattern:', pattern_step) 
                    lights = vex.Color.TRANSPARENT
            return (lights, 0.5)
            
        def start(self,event):
            print(f"CmdFlash:  '{event.data}'")
            spec = event.data
            arg = spec.split(' ',maxsplit=1)[1]
            pattern_steps = re.findall(r'(\w+|(?:\(\w+(?:, \w+)*\)))', arg)
            led_program = [self.program_step(p) for p in pattern_steps]
            self.led_program = led_program
            self.num_cycles = 3
            super().start()


    class CmdEmoji(ShowEmoji):
        def start(self,event):
            print(event.data)
            spec = event.data.split(' ')
            if len(spec) > 0:
                self.emoji = getattr(vex.EmojiType, spec[1], None)
                if self.emoji is None:
                    print(f"Invalid emoji name '{spec[1]}' for #emoji")
                    self.emoji = vex.EmojiType.DISGUST
            else:
                print('No emoji name specified for #emoji')
                self.emoji = vex.EmojiType.WORRIED
            super().start(None)
        

    class CmdAct(StateNode):
        class SendAction(StateNode):
            def start(self, event=None):
                super().start(event)
                self.post_data(self.parent.act)

        def start(self,event):
            print(event.data)
            spec = event.data.split(' ')
            if len(spec) > 0:
                self.act = spec[1]
            else:
                self.act = ''
            super().start()

        def setup(self):
            #             dispatch: self.SendAction()
            #             dispatch =D('happy')=> ActHappy() =C=> complete
            #             dispatch =D('sad')=> ActSad() =C=> complete
            #             dispatch =D('silly')=> ActSilly() =C=> complete
            #             dispatch =D('angry')=> ActAngry() =C=> complete
            #             dispatch =D('excited')=> ActExcited() =C=> complete
            # 
            #             complete: ParentCompletes()
            
            # Code generated by genfsm on Fri Sep 25 22:11:14 2026:
            
            dispatch = self.SendAction() .set_name("dispatch") .set_parent(self)
            acthappy1 = ActHappy() .set_name("acthappy1") .set_parent(self)
            actsad1 = ActSad() .set_name("actsad1") .set_parent(self)
            actsilly1 = ActSilly() .set_name("actsilly1") .set_parent(self)
            actangry1 = ActAngry() .set_name("actangry1") .set_parent(self)
            actexcited1 = ActExcited() .set_name("actexcited1") .set_parent(self)
            complete = ParentCompletes() .set_name("complete") .set_parent(self)
            
            datatrans7 = DataTrans('happy') .set_name("datatrans7")
            datatrans7 .add_sources(dispatch) .add_destinations(acthappy1)
            
            completiontrans17 = CompletionTrans() .set_name("completiontrans17")
            completiontrans17 .add_sources(acthappy1) .add_destinations(complete)
            
            datatrans8 = DataTrans('sad') .set_name("datatrans8")
            datatrans8 .add_sources(dispatch) .add_destinations(actsad1)
            
            completiontrans18 = CompletionTrans() .set_name("completiontrans18")
            completiontrans18 .add_sources(actsad1) .add_destinations(complete)
            
            datatrans9 = DataTrans('silly') .set_name("datatrans9")
            datatrans9 .add_sources(dispatch) .add_destinations(actsilly1)
            
            completiontrans19 = CompletionTrans() .set_name("completiontrans19")
            completiontrans19 .add_sources(actsilly1) .add_destinations(complete)
            
            datatrans10 = DataTrans('angry') .set_name("datatrans10")
            datatrans10 .add_sources(dispatch) .add_destinations(actangry1)
            
            completiontrans20 = CompletionTrans() .set_name("completiontrans20")
            completiontrans20 .add_sources(actangry1) .add_destinations(complete)
            
            datatrans11 = DataTrans('excited') .set_name("datatrans11")
            datatrans11 .add_sources(dispatch) .add_destinations(actexcited1)
            
            completiontrans21 = CompletionTrans() .set_name("completiontrans21")
            completiontrans21 .add_sources(actexcited1) .add_destinations(complete)
            
            return self


    class CmdPlayNotes(PlayNotes):
        def start(self, event):
            print(event.data)
            self.score = event.data[event.data.find(' ')+1:]
            super().start()


    class SpeakResponse(Say):
      def start(self,event):
        self.text = event.data
        super().start(event)
        
    def setup(self):
        #         Print(f"Celeste version {CELESTE_VERSION}") =N=>
        #           TagDetection(True) =T(2)=>
        #             Say("Talk to me") =C=> loop
        # 
        #         putdown: Say(["I'm good", "Okay then", "I'm back", "Now then"]) =C=> loop
        # 
        #         loop: self.Idle() =Hear()=> AskGPT() =OpenAITrans()=> check
        # 
        #         check: self.CheckResponse()
        #         check =D(list)=> dispatch
        #         check =D(str)=> self.SpeakResponse() =C=> loop
        # 
        #         reset_fsm: Say(["I'm awake", "state machine reset", "I'm back on the job",
        #                         "I'm ready for you", "You have my attention",
        #                         "I'm listening", "Let's go", "I'm back"]) =C=> loop
        # 
        #         dispatch: Iterate()
        #         dispatch =D(re.compile('#hang$'))=> self.CmdHang()
        #         dispatch =D(re.compile('#script '))=> script
        #         dispatch =D(re.compile('#say '))=> say
        #         dispatch =D(re.compile('#forward '))=> self.CmdForward() =CNext=> dispatch
        #         dispatch =D(re.compile('#sideways '))=> self.CmdSideways() =CNext=> dispatch
        #         dispatch =D(re.compile('#turn '))=> self.CmdTurn() =CNext=> dispatch
        #         dispatch =D(re.compile('#turntoward '))=> turntoward
        #         dispatch =D(re.compile('#search '))=> search
        #         dispatch =D(re.compile('#pilottoobject '))=> pilottoobject
        #         dispatch =D(re.compile(r'^#pilottopose(?:\s|$)'))=> pilottopose
        #         dispatch =D(re.compile('#doorpass '))=> doorpass
        #         dispatch =D(re.compile('#pickup '))=> pickup
        #         dispatch =D(re.compile('#rename '))=> rename
        #         rename: self.CmdRename()
        #         rename =CNext=> dispatch
        #         rename =F=> self.CmdFailed('The rename failed; no object was renamed. Tell the user briefly and ask which object they mean, using everyday language rather than internal map IDs.', ends_request=True) =OpenAITrans()=> check
        #         dispatch =D(re.compile('#find '))=> announcesearch
        #         announcesearch: self.AnnounceFind() =D()=> find
        #         # Allow detection results to reach the map before reporting.
        #         find: self.CmdFind()
        #         find =S=> StateNode() =Next=> dispatch
        #         find =F=> self.CmdFailed("I could not find %s.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         find =D('scan')=> Say('I am turning to search another view.') =C=> findturn
        #         find =D('retry')=> Say('I am checking again without moving.') =C=> StateNode() =T(self.CmdFind.WAIT_SECS)=> find
        #         find =D('reframe')=> Say('I am turning to bring the object into view.') =C=> reframefind
        #         find =D('backup')=> Say('I am backing up to see its base.') =C=> findbackup
        #         findbackup: Forward(-BASE_BACKUP_MM)
        #         findbackup =C=> StateNode() =T(self.CmdFind.WAIT_SECS)=> find
        #         findbackup =F=> self.CmdFailed("I could not back up from %s.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         reframefind: self.ReframeFind()
        #         reframefind =C=> StateNode() =T(self.CmdFind.WAIT_SECS)=> find
        #         reframefind =F=> self.CmdFailed("I could not turn toward %s.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         findturn: Turn(self.CmdFind.TURN_STEP)
        #         findturn =C=> StateNode() =T(self.CmdFind.WAIT_SECS)=> find
        #         findturn =F=> self.CmdFailed("My turn while searching for %s failed.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         find =D('unmappable')=> self.CmdFailed("I can see %s but I could not map its location.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         find =T(DETECTION_TIMEOUT)=> self.CmdFailed("Searching for %s took too long.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         find =C=> Say('I have a location estimate. I am checking the map.') =C=> StateNode() =T(self.CmdFind.WAIT_SECS)=> findreport
        #         findreport: self.CmdFindReport()
        #         findreport =CNext=> dispatch
        #         findreport =F=> self.CmdFailed("I could not locate %s precisely enough to use it.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         findreport =D('approach')=> Say('I will move closer to map the target.') =C=> mapapproach
        #         mapapproach: self.MapOpenVocabTarget()
        #         mapapproach =C=> findreport
        #         mapapproach =F=> self.CmdFailed("I could not get close enough to %s to locate it.", lambda : find.target, ends_request=True) =OpenAITrans()=> check
        #         dispatch =D(re.compile('#drop$'))=> self.CmdDrop() =CNext=> dispatch
        #         dispatch =D(re.compile('#kick$'))=> preparekick
        #         preparekick: self.PrepareKick()
        #         preparekick =C=> kick
        #         # Open-vocabulary close-contact behavior is disabled pending robot testing.
        #         preparekick =S=> Say('Close-contact actions for these objects are not enabled yet.') =CNext=> dispatch
        #         # preparekick =S=> contactapproach
        #         # contactapproach: self.OpenVocabApproach()
        #         # contactapproach =C=> kick
        #         # contactapproach =F=> self.CmdFailed("I could not get into position to kick.") =OpenAITrans()=> check
        #         # contactapproach =D('wrong')=> kickretarget
        #         # kickretarget: self.Retarget(then_kick=True)
        #         # kickretarget =C=> StateNode() =Next=> dispatch
        #         # kickretarget =F=> self.CmdFailed("The object I reached was not %s.", lambda : getattr(self.navigation_target, "label", "the requested object"), ends_request=True) =OpenAITrans()=> check
        #         kick: self.CmdKick() =CNext=> dispatch
        #         dispatch =D(re.compile('#glow '))=> self.CmdGlow() =CNext=> dispatch
        #         dispatch =D(re.compile('#flash '))=> self.CmdFlash() =CNext=> dispatch
        #         dispatch =D(re.compile('#emoji '))=> self.CmdEmoji() =CNext=> dispatch
        #         dispatch =D(re.compile('#act '))=> self.CmdAct() =CNext=> dispatch
        #         dispatch =D(re.compile('#playnotes '))=> self.CmdPlayNotes() =CNext=> dispatch
        #         dispatch =D(re.compile('#camera$'))=> SendGPTCamera() =C=>
        #             AskGPT("Please respond to the query using the camera image.") =OpenAITrans()=> check
        #         dispatch =D()=> Print(prefix='Unrecognized #-command: ') =Next=> dispatch
        #         dispatch =C=> loop
        # 
        #         say: self.CmdSay() =CNext=> dispatch
        #         say =T(5)=> StateNode() =Next=> dispatch  # safety
        # 
        #         script: self.CmdScript()
        #         script =C=> Say("Script loaded") =CNext=> dispatch
        #         script =F=> self.CmdFailed("I am unable to load that file") =OpenAITrans()=> check
        #         script =T(5)=> StateNode() =Next=> dispatch  # safety
        # 
        #         turntoward: self.CmdTurnToward()
        #         turntoward =CNext=> dispatch
        #         turntoward =F=> StateNode() =Next=> dispatch
        # 
        #         pilottoobject: self.CmdPilotToObject()
        #         # Arrival recognition never commands close-contact movement.
        #         pilottoobject =C=> arrivalcheck
        #         arrivalcheck: self.CheckArrival()
        #         arrivalcheck =C=> arrived
        #         arrivalcheck =D('absent')=> self.AnnounceRelocation() =C=> arrivalretry
        #         arrivalretry: self.Retarget()
        #         arrivalretry =C=> StateNode() =Next=> dispatch
        #         arrivalretry =F=> self.CmdFailed('I could not confirm the object after searching again.', ends_request=True) =OpenAITrans()=> check
        #         arrivalcheck =F=> self.CmdFailed('I reached the mapped position but could not confirm the object in the camera.', ends_request=True) =OpenAITrans()=> check
        #         arrivalcheck =T(DETECTION_TIMEOUT)=> self.CmdFailed('Checking the object at arrival took too long.', ends_request=True) =OpenAITrans()=> check
        #         arrived: self.CaptureArrival() =CNext=> dispatch
        #         pilottoobject =PILOT(GoalUnreachable)=>
        #             self.CmdFailed("The object %s is not reachable due to obstructions", lambda : pilottoobject.object_spec) =OpenAITrans()=> check
        #         pilottoobject =F=>
        #             self.CmdFailed("The name '%s' is not a valid object name.", lambda : pilottoobject.object_spec) =OpenAITrans()=> check
        # 
        #         pilottopose: self.CmdPilotToPose()
        #         pilottopose =C=> StateNode() =Next=> dispatch
        #         pilottopose =F=> self.CmdFailed("I could not drive to that position.") =OpenAITrans()=> check
        # 
        #         doorpass: self.CmdDoorPass()
        #         doorpass =CNext=> dispatch
        #         doorpass =F=> self.CmdFailed("Doorpass failed for '%s'", lambda : doorpass.door_spec) =OpenAITrans()=> check
        # 
        #         pickup: self.CmdPickup()
        #         pickup =CNext=> dispatch
        #         pickup =F=> StateNode() =Next=> dispatch
        # 
        #         search: self.CmdSearch()
        #         search =CNext=> dispatch
        #         search =F=> self.CmdFailed(self.SEARCH_FAIL_MSG, lambda : search.object_spec) =OpenAITrans()=> check
        # 
        
        # Code generated by genfsm on Fri Sep 25 22:11:14 2026:
        
        print2 = Print(f"Celeste version {CELESTE_VERSION}") .set_name("print2") .set_parent(self)
        tagdetection1 = TagDetection(True) .set_name("tagdetection1") .set_parent(self)
        say5 = Say("Talk to me") .set_name("say5") .set_parent(self)
        putdown = Say(["I'm good", "Okay then", "I'm back", "Now then"]) .set_name("putdown") .set_parent(self)
        loop = self.Idle() .set_name("loop") .set_parent(self)
        askgpt1 = AskGPT() .set_name("askgpt1") .set_parent(self)
        check = self.CheckResponse() .set_name("check") .set_parent(self)
        speakresponse1 = self.SpeakResponse() .set_name("speakresponse1") .set_parent(self)
        reset_fsm = Say(["I'm awake", "state machine reset", "I'm back on the job",
                        "I'm ready for you", "You have my attention",
                        "I'm listening", "Let's go", "I'm back"]) .set_name("reset_fsm") .set_parent(self)
        dispatch = Iterate() .set_name("dispatch") .set_parent(self)
        cmdhang1 = self.CmdHang() .set_name("cmdhang1") .set_parent(self)
        cmdforward1 = self.CmdForward() .set_name("cmdforward1") .set_parent(self)
        cmdsideways1 = self.CmdSideways() .set_name("cmdsideways1") .set_parent(self)
        cmdturn1 = self.CmdTurn() .set_name("cmdturn1") .set_parent(self)
        rename = self.CmdRename() .set_name("rename") .set_parent(self)
        cmdfailed1 = self.CmdFailed('The rename failed; no object was renamed. Tell the user briefly and ask which object they mean, using everyday language rather than internal map IDs.', ends_request=True) .set_name("cmdfailed1") .set_parent(self)
        announcesearch = self.AnnounceFind() .set_name("announcesearch") .set_parent(self)
        find = self.CmdFind() .set_name("find") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        cmdfailed2 = self.CmdFailed("I could not find %s.", lambda : find.target, ends_request=True) .set_name("cmdfailed2") .set_parent(self)
        say6 = Say('I am turning to search another view.') .set_name("say6") .set_parent(self)
        say7 = Say('I am checking again without moving.') .set_name("say7") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        say8 = Say('I am turning to bring the object into view.') .set_name("say8") .set_parent(self)
        say9 = Say('I am backing up to see its base.') .set_name("say9") .set_parent(self)
        findbackup = Forward(-BASE_BACKUP_MM) .set_name("findbackup") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        cmdfailed3 = self.CmdFailed("I could not back up from %s.", lambda : find.target, ends_request=True) .set_name("cmdfailed3") .set_parent(self)
        reframefind = self.ReframeFind() .set_name("reframefind") .set_parent(self)
        statenode4 = StateNode() .set_name("statenode4") .set_parent(self)
        cmdfailed4 = self.CmdFailed("I could not turn toward %s.", lambda : find.target, ends_request=True) .set_name("cmdfailed4") .set_parent(self)
        findturn = Turn(self.CmdFind.TURN_STEP) .set_name("findturn") .set_parent(self)
        statenode5 = StateNode() .set_name("statenode5") .set_parent(self)
        cmdfailed5 = self.CmdFailed("My turn while searching for %s failed.", lambda : find.target, ends_request=True) .set_name("cmdfailed5") .set_parent(self)
        cmdfailed6 = self.CmdFailed("I can see %s but I could not map its location.", lambda : find.target, ends_request=True) .set_name("cmdfailed6") .set_parent(self)
        cmdfailed7 = self.CmdFailed("Searching for %s took too long.", lambda : find.target, ends_request=True) .set_name("cmdfailed7") .set_parent(self)
        say10 = Say('I have a location estimate. I am checking the map.') .set_name("say10") .set_parent(self)
        statenode6 = StateNode() .set_name("statenode6") .set_parent(self)
        findreport = self.CmdFindReport() .set_name("findreport") .set_parent(self)
        cmdfailed8 = self.CmdFailed("I could not locate %s precisely enough to use it.", lambda : find.target, ends_request=True) .set_name("cmdfailed8") .set_parent(self)
        say11 = Say('I will move closer to map the target.') .set_name("say11") .set_parent(self)
        mapapproach = self.MapOpenVocabTarget() .set_name("mapapproach") .set_parent(self)
        cmdfailed9 = self.CmdFailed("I could not get close enough to %s to locate it.", lambda : find.target, ends_request=True) .set_name("cmdfailed9") .set_parent(self)
        cmddrop1 = self.CmdDrop() .set_name("cmddrop1") .set_parent(self)
        preparekick = self.PrepareKick() .set_name("preparekick") .set_parent(self)
        say12 = Say('Close-contact actions for these objects are not enabled yet.') .set_name("say12") .set_parent(self)
        kick = self.CmdKick() .set_name("kick") .set_parent(self)
        cmdglow1 = self.CmdGlow() .set_name("cmdglow1") .set_parent(self)
        cmdflash1 = self.CmdFlash() .set_name("cmdflash1") .set_parent(self)
        cmdemoji1 = self.CmdEmoji() .set_name("cmdemoji1") .set_parent(self)
        cmdact1 = self.CmdAct() .set_name("cmdact1") .set_parent(self)
        cmdplaynotes1 = self.CmdPlayNotes() .set_name("cmdplaynotes1") .set_parent(self)
        sendgptcamera1 = SendGPTCamera() .set_name("sendgptcamera1") .set_parent(self)
        askgpt2 = AskGPT("Please respond to the query using the camera image.") .set_name("askgpt2") .set_parent(self)
        print3 = Print(prefix='Unrecognized #-command: ') .set_name("print3") .set_parent(self)
        say = self.CmdSay() .set_name("say") .set_parent(self)
        statenode7 = StateNode() .set_name("statenode7") .set_parent(self)
        script = self.CmdScript() .set_name("script") .set_parent(self)
        say13 = Say("Script loaded") .set_name("say13") .set_parent(self)
        cmdfailed10 = self.CmdFailed("I am unable to load that file") .set_name("cmdfailed10") .set_parent(self)
        statenode8 = StateNode() .set_name("statenode8") .set_parent(self)
        turntoward = self.CmdTurnToward() .set_name("turntoward") .set_parent(self)
        statenode9 = StateNode() .set_name("statenode9") .set_parent(self)
        pilottoobject = self.CmdPilotToObject() .set_name("pilottoobject") .set_parent(self)
        arrivalcheck = self.CheckArrival() .set_name("arrivalcheck") .set_parent(self)
        announcerelocation1 = self.AnnounceRelocation() .set_name("announcerelocation1") .set_parent(self)
        arrivalretry = self.Retarget() .set_name("arrivalretry") .set_parent(self)
        statenode10 = StateNode() .set_name("statenode10") .set_parent(self)
        cmdfailed11 = self.CmdFailed('I could not confirm the object after searching again.', ends_request=True) .set_name("cmdfailed11") .set_parent(self)
        cmdfailed12 = self.CmdFailed('I reached the mapped position but could not confirm the object in the camera.', ends_request=True) .set_name("cmdfailed12") .set_parent(self)
        cmdfailed13 = self.CmdFailed('Checking the object at arrival took too long.', ends_request=True) .set_name("cmdfailed13") .set_parent(self)
        arrived = self.CaptureArrival() .set_name("arrived") .set_parent(self)
        cmdfailed14 = self.CmdFailed("The object %s is not reachable due to obstructions", lambda : pilottoobject.object_spec) .set_name("cmdfailed14") .set_parent(self)
        cmdfailed15 = self.CmdFailed("The name '%s' is not a valid object name.", lambda : pilottoobject.object_spec) .set_name("cmdfailed15") .set_parent(self)
        pilottopose = self.CmdPilotToPose() .set_name("pilottopose") .set_parent(self)
        statenode11 = StateNode() .set_name("statenode11") .set_parent(self)
        cmdfailed16 = self.CmdFailed("I could not drive to that position.") .set_name("cmdfailed16") .set_parent(self)
        doorpass = self.CmdDoorPass() .set_name("doorpass") .set_parent(self)
        cmdfailed17 = self.CmdFailed("Doorpass failed for '%s'", lambda : doorpass.door_spec) .set_name("cmdfailed17") .set_parent(self)
        pickup = self.CmdPickup() .set_name("pickup") .set_parent(self)
        statenode12 = StateNode() .set_name("statenode12") .set_parent(self)
        search = self.CmdSearch() .set_name("search") .set_parent(self)
        cmdfailed18 = self.CmdFailed(self.SEARCH_FAIL_MSG, lambda : search.object_spec) .set_name("cmdfailed18") .set_parent(self)
        
        nulltrans2 = NullTrans() .set_name("nulltrans2")
        nulltrans2 .add_sources(print2) .add_destinations(tagdetection1)
        
        timertrans5 = TimerTrans(2) .set_name("timertrans5")
        timertrans5 .add_sources(tagdetection1) .add_destinations(say5)
        
        completiontrans22 = CompletionTrans() .set_name("completiontrans22")
        completiontrans22 .add_sources(say5) .add_destinations(loop)
        
        completiontrans23 = CompletionTrans() .set_name("completiontrans23")
        completiontrans23 .add_sources(putdown) .add_destinations(loop)
        
        heartrans1 = HearTrans() .set_name("heartrans1")
        heartrans1 .add_sources(loop) .add_destinations(askgpt1)
        
        openaitrans1 = OpenAITrans() .set_name("openaitrans1")
        openaitrans1 .add_sources(askgpt1) .add_destinations(check)
        
        datatrans12 = DataTrans(list) .set_name("datatrans12")
        datatrans12 .add_sources(check) .add_destinations(dispatch)
        
        datatrans13 = DataTrans(str) .set_name("datatrans13")
        datatrans13 .add_sources(check) .add_destinations(speakresponse1)
        
        completiontrans24 = CompletionTrans() .set_name("completiontrans24")
        completiontrans24 .add_sources(speakresponse1) .add_destinations(loop)
        
        completiontrans25 = CompletionTrans() .set_name("completiontrans25")
        completiontrans25 .add_sources(reset_fsm) .add_destinations(loop)
        
        datatrans14 = DataTrans(re.compile('#hang$')) .set_name("datatrans14")
        datatrans14 .add_sources(dispatch) .add_destinations(cmdhang1)
        
        datatrans15 = DataTrans(re.compile('#script ')) .set_name("datatrans15")
        datatrans15 .add_sources(dispatch) .add_destinations(script)
        
        datatrans16 = DataTrans(re.compile('#say ')) .set_name("datatrans16")
        datatrans16 .add_sources(dispatch) .add_destinations(say)
        
        datatrans17 = DataTrans(re.compile('#forward ')) .set_name("datatrans17")
        datatrans17 .add_sources(dispatch) .add_destinations(cmdforward1)
        
        cnexttrans1 = CNextTrans() .set_name("cnexttrans1")
        cnexttrans1 .add_sources(cmdforward1) .add_destinations(dispatch)
        
        datatrans18 = DataTrans(re.compile('#sideways ')) .set_name("datatrans18")
        datatrans18 .add_sources(dispatch) .add_destinations(cmdsideways1)
        
        cnexttrans2 = CNextTrans() .set_name("cnexttrans2")
        cnexttrans2 .add_sources(cmdsideways1) .add_destinations(dispatch)
        
        datatrans19 = DataTrans(re.compile('#turn ')) .set_name("datatrans19")
        datatrans19 .add_sources(dispatch) .add_destinations(cmdturn1)
        
        cnexttrans3 = CNextTrans() .set_name("cnexttrans3")
        cnexttrans3 .add_sources(cmdturn1) .add_destinations(dispatch)
        
        datatrans20 = DataTrans(re.compile('#turntoward ')) .set_name("datatrans20")
        datatrans20 .add_sources(dispatch) .add_destinations(turntoward)
        
        datatrans21 = DataTrans(re.compile('#search ')) .set_name("datatrans21")
        datatrans21 .add_sources(dispatch) .add_destinations(search)
        
        datatrans22 = DataTrans(re.compile('#pilottoobject ')) .set_name("datatrans22")
        datatrans22 .add_sources(dispatch) .add_destinations(pilottoobject)
        
        datatrans23 = DataTrans(re.compile(r'^#pilottopose(?:\s|$)')) .set_name("datatrans23")
        datatrans23 .add_sources(dispatch) .add_destinations(pilottopose)
        
        datatrans24 = DataTrans(re.compile('#doorpass ')) .set_name("datatrans24")
        datatrans24 .add_sources(dispatch) .add_destinations(doorpass)
        
        datatrans25 = DataTrans(re.compile('#pickup ')) .set_name("datatrans25")
        datatrans25 .add_sources(dispatch) .add_destinations(pickup)
        
        datatrans26 = DataTrans(re.compile('#rename ')) .set_name("datatrans26")
        datatrans26 .add_sources(dispatch) .add_destinations(rename)
        
        cnexttrans4 = CNextTrans() .set_name("cnexttrans4")
        cnexttrans4 .add_sources(rename) .add_destinations(dispatch)
        
        failuretrans14 = FailureTrans() .set_name("failuretrans14")
        failuretrans14 .add_sources(rename) .add_destinations(cmdfailed1)
        
        openaitrans2 = OpenAITrans() .set_name("openaitrans2")
        openaitrans2 .add_sources(cmdfailed1) .add_destinations(check)
        
        datatrans27 = DataTrans(re.compile('#find ')) .set_name("datatrans27")
        datatrans27 .add_sources(dispatch) .add_destinations(announcesearch)
        
        datatrans28 = DataTrans() .set_name("datatrans28")
        datatrans28 .add_sources(announcesearch) .add_destinations(find)
        
        successtrans4 = SuccessTrans() .set_name("successtrans4")
        successtrans4 .add_sources(find) .add_destinations(statenode1)
        
        nexttrans1 = NextTrans() .set_name("nexttrans1")
        nexttrans1 .add_sources(statenode1) .add_destinations(dispatch)
        
        failuretrans15 = FailureTrans() .set_name("failuretrans15")
        failuretrans15 .add_sources(find) .add_destinations(cmdfailed2)
        
        openaitrans3 = OpenAITrans() .set_name("openaitrans3")
        openaitrans3 .add_sources(cmdfailed2) .add_destinations(check)
        
        datatrans29 = DataTrans('scan') .set_name("datatrans29")
        datatrans29 .add_sources(find) .add_destinations(say6)
        
        completiontrans26 = CompletionTrans() .set_name("completiontrans26")
        completiontrans26 .add_sources(say6) .add_destinations(findturn)
        
        datatrans30 = DataTrans('retry') .set_name("datatrans30")
        datatrans30 .add_sources(find) .add_destinations(say7)
        
        completiontrans27 = CompletionTrans() .set_name("completiontrans27")
        completiontrans27 .add_sources(say7) .add_destinations(statenode2)
        
        timertrans6 = TimerTrans(self.CmdFind.WAIT_SECS) .set_name("timertrans6")
        timertrans6 .add_sources(statenode2) .add_destinations(find)
        
        datatrans31 = DataTrans('reframe') .set_name("datatrans31")
        datatrans31 .add_sources(find) .add_destinations(say8)
        
        completiontrans28 = CompletionTrans() .set_name("completiontrans28")
        completiontrans28 .add_sources(say8) .add_destinations(reframefind)
        
        datatrans32 = DataTrans('backup') .set_name("datatrans32")
        datatrans32 .add_sources(find) .add_destinations(say9)
        
        completiontrans29 = CompletionTrans() .set_name("completiontrans29")
        completiontrans29 .add_sources(say9) .add_destinations(findbackup)
        
        completiontrans30 = CompletionTrans() .set_name("completiontrans30")
        completiontrans30 .add_sources(findbackup) .add_destinations(statenode3)
        
        timertrans7 = TimerTrans(self.CmdFind.WAIT_SECS) .set_name("timertrans7")
        timertrans7 .add_sources(statenode3) .add_destinations(find)
        
        failuretrans16 = FailureTrans() .set_name("failuretrans16")
        failuretrans16 .add_sources(findbackup) .add_destinations(cmdfailed3)
        
        openaitrans4 = OpenAITrans() .set_name("openaitrans4")
        openaitrans4 .add_sources(cmdfailed3) .add_destinations(check)
        
        completiontrans31 = CompletionTrans() .set_name("completiontrans31")
        completiontrans31 .add_sources(reframefind) .add_destinations(statenode4)
        
        timertrans8 = TimerTrans(self.CmdFind.WAIT_SECS) .set_name("timertrans8")
        timertrans8 .add_sources(statenode4) .add_destinations(find)
        
        failuretrans17 = FailureTrans() .set_name("failuretrans17")
        failuretrans17 .add_sources(reframefind) .add_destinations(cmdfailed4)
        
        openaitrans5 = OpenAITrans() .set_name("openaitrans5")
        openaitrans5 .add_sources(cmdfailed4) .add_destinations(check)
        
        completiontrans32 = CompletionTrans() .set_name("completiontrans32")
        completiontrans32 .add_sources(findturn) .add_destinations(statenode5)
        
        timertrans9 = TimerTrans(self.CmdFind.WAIT_SECS) .set_name("timertrans9")
        timertrans9 .add_sources(statenode5) .add_destinations(find)
        
        failuretrans18 = FailureTrans() .set_name("failuretrans18")
        failuretrans18 .add_sources(findturn) .add_destinations(cmdfailed5)
        
        openaitrans6 = OpenAITrans() .set_name("openaitrans6")
        openaitrans6 .add_sources(cmdfailed5) .add_destinations(check)
        
        datatrans33 = DataTrans('unmappable') .set_name("datatrans33")
        datatrans33 .add_sources(find) .add_destinations(cmdfailed6)
        
        openaitrans7 = OpenAITrans() .set_name("openaitrans7")
        openaitrans7 .add_sources(cmdfailed6) .add_destinations(check)
        
        timertrans10 = TimerTrans(DETECTION_TIMEOUT) .set_name("timertrans10")
        timertrans10 .add_sources(find) .add_destinations(cmdfailed7)
        
        openaitrans8 = OpenAITrans() .set_name("openaitrans8")
        openaitrans8 .add_sources(cmdfailed7) .add_destinations(check)
        
        completiontrans33 = CompletionTrans() .set_name("completiontrans33")
        completiontrans33 .add_sources(find) .add_destinations(say10)
        
        completiontrans34 = CompletionTrans() .set_name("completiontrans34")
        completiontrans34 .add_sources(say10) .add_destinations(statenode6)
        
        timertrans11 = TimerTrans(self.CmdFind.WAIT_SECS) .set_name("timertrans11")
        timertrans11 .add_sources(statenode6) .add_destinations(findreport)
        
        cnexttrans5 = CNextTrans() .set_name("cnexttrans5")
        cnexttrans5 .add_sources(findreport) .add_destinations(dispatch)
        
        failuretrans19 = FailureTrans() .set_name("failuretrans19")
        failuretrans19 .add_sources(findreport) .add_destinations(cmdfailed8)
        
        openaitrans9 = OpenAITrans() .set_name("openaitrans9")
        openaitrans9 .add_sources(cmdfailed8) .add_destinations(check)
        
        datatrans34 = DataTrans('approach') .set_name("datatrans34")
        datatrans34 .add_sources(findreport) .add_destinations(say11)
        
        completiontrans35 = CompletionTrans() .set_name("completiontrans35")
        completiontrans35 .add_sources(say11) .add_destinations(mapapproach)
        
        completiontrans36 = CompletionTrans() .set_name("completiontrans36")
        completiontrans36 .add_sources(mapapproach) .add_destinations(findreport)
        
        failuretrans20 = FailureTrans() .set_name("failuretrans20")
        failuretrans20 .add_sources(mapapproach) .add_destinations(cmdfailed9)
        
        openaitrans10 = OpenAITrans() .set_name("openaitrans10")
        openaitrans10 .add_sources(cmdfailed9) .add_destinations(check)
        
        datatrans35 = DataTrans(re.compile('#drop$')) .set_name("datatrans35")
        datatrans35 .add_sources(dispatch) .add_destinations(cmddrop1)
        
        cnexttrans6 = CNextTrans() .set_name("cnexttrans6")
        cnexttrans6 .add_sources(cmddrop1) .add_destinations(dispatch)
        
        datatrans36 = DataTrans(re.compile('#kick$')) .set_name("datatrans36")
        datatrans36 .add_sources(dispatch) .add_destinations(preparekick)
        
        completiontrans37 = CompletionTrans() .set_name("completiontrans37")
        completiontrans37 .add_sources(preparekick) .add_destinations(kick)
        
        successtrans5 = SuccessTrans() .set_name("successtrans5")
        successtrans5 .add_sources(preparekick) .add_destinations(say12)
        
        cnexttrans7 = CNextTrans() .set_name("cnexttrans7")
        cnexttrans7 .add_sources(say12) .add_destinations(dispatch)
        
        cnexttrans8 = CNextTrans() .set_name("cnexttrans8")
        cnexttrans8 .add_sources(kick) .add_destinations(dispatch)
        
        datatrans37 = DataTrans(re.compile('#glow ')) .set_name("datatrans37")
        datatrans37 .add_sources(dispatch) .add_destinations(cmdglow1)
        
        cnexttrans9 = CNextTrans() .set_name("cnexttrans9")
        cnexttrans9 .add_sources(cmdglow1) .add_destinations(dispatch)
        
        datatrans38 = DataTrans(re.compile('#flash ')) .set_name("datatrans38")
        datatrans38 .add_sources(dispatch) .add_destinations(cmdflash1)
        
        cnexttrans10 = CNextTrans() .set_name("cnexttrans10")
        cnexttrans10 .add_sources(cmdflash1) .add_destinations(dispatch)
        
        datatrans39 = DataTrans(re.compile('#emoji ')) .set_name("datatrans39")
        datatrans39 .add_sources(dispatch) .add_destinations(cmdemoji1)
        
        cnexttrans11 = CNextTrans() .set_name("cnexttrans11")
        cnexttrans11 .add_sources(cmdemoji1) .add_destinations(dispatch)
        
        datatrans40 = DataTrans(re.compile('#act ')) .set_name("datatrans40")
        datatrans40 .add_sources(dispatch) .add_destinations(cmdact1)
        
        cnexttrans12 = CNextTrans() .set_name("cnexttrans12")
        cnexttrans12 .add_sources(cmdact1) .add_destinations(dispatch)
        
        datatrans41 = DataTrans(re.compile('#playnotes ')) .set_name("datatrans41")
        datatrans41 .add_sources(dispatch) .add_destinations(cmdplaynotes1)
        
        cnexttrans13 = CNextTrans() .set_name("cnexttrans13")
        cnexttrans13 .add_sources(cmdplaynotes1) .add_destinations(dispatch)
        
        datatrans42 = DataTrans(re.compile('#camera$')) .set_name("datatrans42")
        datatrans42 .add_sources(dispatch) .add_destinations(sendgptcamera1)
        
        completiontrans38 = CompletionTrans() .set_name("completiontrans38")
        completiontrans38 .add_sources(sendgptcamera1) .add_destinations(askgpt2)
        
        openaitrans11 = OpenAITrans() .set_name("openaitrans11")
        openaitrans11 .add_sources(askgpt2) .add_destinations(check)
        
        datatrans43 = DataTrans() .set_name("datatrans43")
        datatrans43 .add_sources(dispatch) .add_destinations(print3)
        
        nexttrans2 = NextTrans() .set_name("nexttrans2")
        nexttrans2 .add_sources(print3) .add_destinations(dispatch)
        
        completiontrans39 = CompletionTrans() .set_name("completiontrans39")
        completiontrans39 .add_sources(dispatch) .add_destinations(loop)
        
        cnexttrans14 = CNextTrans() .set_name("cnexttrans14")
        cnexttrans14 .add_sources(say) .add_destinations(dispatch)
        
        timertrans12 = TimerTrans(5) .set_name("timertrans12")
        timertrans12 .add_sources(say) .add_destinations(statenode7)
        
        nexttrans3 = NextTrans() .set_name("nexttrans3")
        nexttrans3 .add_sources(statenode7) .add_destinations(dispatch)
        
        completiontrans40 = CompletionTrans() .set_name("completiontrans40")
        completiontrans40 .add_sources(script) .add_destinations(say13)
        
        cnexttrans15 = CNextTrans() .set_name("cnexttrans15")
        cnexttrans15 .add_sources(say13) .add_destinations(dispatch)
        
        failuretrans21 = FailureTrans() .set_name("failuretrans21")
        failuretrans21 .add_sources(script) .add_destinations(cmdfailed10)
        
        openaitrans12 = OpenAITrans() .set_name("openaitrans12")
        openaitrans12 .add_sources(cmdfailed10) .add_destinations(check)
        
        timertrans13 = TimerTrans(5) .set_name("timertrans13")
        timertrans13 .add_sources(script) .add_destinations(statenode8)
        
        nexttrans4 = NextTrans() .set_name("nexttrans4")
        nexttrans4 .add_sources(statenode8) .add_destinations(dispatch)
        
        cnexttrans16 = CNextTrans() .set_name("cnexttrans16")
        cnexttrans16 .add_sources(turntoward) .add_destinations(dispatch)
        
        failuretrans22 = FailureTrans() .set_name("failuretrans22")
        failuretrans22 .add_sources(turntoward) .add_destinations(statenode9)
        
        nexttrans5 = NextTrans() .set_name("nexttrans5")
        nexttrans5 .add_sources(statenode9) .add_destinations(dispatch)
        
        completiontrans41 = CompletionTrans() .set_name("completiontrans41")
        completiontrans41 .add_sources(pilottoobject) .add_destinations(arrivalcheck)
        
        completiontrans42 = CompletionTrans() .set_name("completiontrans42")
        completiontrans42 .add_sources(arrivalcheck) .add_destinations(arrived)
        
        datatrans44 = DataTrans('absent') .set_name("datatrans44")
        datatrans44 .add_sources(arrivalcheck) .add_destinations(announcerelocation1)
        
        completiontrans43 = CompletionTrans() .set_name("completiontrans43")
        completiontrans43 .add_sources(announcerelocation1) .add_destinations(arrivalretry)
        
        completiontrans44 = CompletionTrans() .set_name("completiontrans44")
        completiontrans44 .add_sources(arrivalretry) .add_destinations(statenode10)
        
        nexttrans6 = NextTrans() .set_name("nexttrans6")
        nexttrans6 .add_sources(statenode10) .add_destinations(dispatch)
        
        failuretrans23 = FailureTrans() .set_name("failuretrans23")
        failuretrans23 .add_sources(arrivalretry) .add_destinations(cmdfailed11)
        
        openaitrans13 = OpenAITrans() .set_name("openaitrans13")
        openaitrans13 .add_sources(cmdfailed11) .add_destinations(check)
        
        failuretrans24 = FailureTrans() .set_name("failuretrans24")
        failuretrans24 .add_sources(arrivalcheck) .add_destinations(cmdfailed12)
        
        openaitrans14 = OpenAITrans() .set_name("openaitrans14")
        openaitrans14 .add_sources(cmdfailed12) .add_destinations(check)
        
        timertrans14 = TimerTrans(DETECTION_TIMEOUT) .set_name("timertrans14")
        timertrans14 .add_sources(arrivalcheck) .add_destinations(cmdfailed13)
        
        openaitrans15 = OpenAITrans() .set_name("openaitrans15")
        openaitrans15 .add_sources(cmdfailed13) .add_destinations(check)
        
        cnexttrans17 = CNextTrans() .set_name("cnexttrans17")
        cnexttrans17 .add_sources(arrived) .add_destinations(dispatch)
        
        pilottrans3 = PilotTrans(GoalUnreachable) .set_name("pilottrans3")
        pilottrans3 .add_sources(pilottoobject) .add_destinations(cmdfailed14)
        
        openaitrans16 = OpenAITrans() .set_name("openaitrans16")
        openaitrans16 .add_sources(cmdfailed14) .add_destinations(check)
        
        failuretrans25 = FailureTrans() .set_name("failuretrans25")
        failuretrans25 .add_sources(pilottoobject) .add_destinations(cmdfailed15)
        
        openaitrans17 = OpenAITrans() .set_name("openaitrans17")
        openaitrans17 .add_sources(cmdfailed15) .add_destinations(check)
        
        completiontrans45 = CompletionTrans() .set_name("completiontrans45")
        completiontrans45 .add_sources(pilottopose) .add_destinations(statenode11)
        
        nexttrans7 = NextTrans() .set_name("nexttrans7")
        nexttrans7 .add_sources(statenode11) .add_destinations(dispatch)
        
        failuretrans26 = FailureTrans() .set_name("failuretrans26")
        failuretrans26 .add_sources(pilottopose) .add_destinations(cmdfailed16)
        
        openaitrans18 = OpenAITrans() .set_name("openaitrans18")
        openaitrans18 .add_sources(cmdfailed16) .add_destinations(check)
        
        cnexttrans18 = CNextTrans() .set_name("cnexttrans18")
        cnexttrans18 .add_sources(doorpass) .add_destinations(dispatch)
        
        failuretrans27 = FailureTrans() .set_name("failuretrans27")
        failuretrans27 .add_sources(doorpass) .add_destinations(cmdfailed17)
        
        openaitrans19 = OpenAITrans() .set_name("openaitrans19")
        openaitrans19 .add_sources(cmdfailed17) .add_destinations(check)
        
        cnexttrans19 = CNextTrans() .set_name("cnexttrans19")
        cnexttrans19 .add_sources(pickup) .add_destinations(dispatch)
        
        failuretrans28 = FailureTrans() .set_name("failuretrans28")
        failuretrans28 .add_sources(pickup) .add_destinations(statenode12)
        
        nexttrans8 = NextTrans() .set_name("nexttrans8")
        nexttrans8 .add_sources(statenode12) .add_destinations(dispatch)
        
        cnexttrans20 = CNextTrans() .set_name("cnexttrans20")
        cnexttrans20 .add_sources(search) .add_destinations(dispatch)
        
        failuretrans29 = FailureTrans() .set_name("failuretrans29")
        failuretrans29 .add_sources(search) .add_destinations(cmdfailed18)
        
        openaitrans20 = OpenAITrans() .set_name("openaitrans20")
        openaitrans20 .add_sources(cmdfailed18) .add_destinations(check)
        
        return self
