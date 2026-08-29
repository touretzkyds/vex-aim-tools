from pathlib import Path

from aim_fsm import *

CELESTE_VERSION = "1.5"

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
  To pick up object X, output the string "#pickup X" without quotes.
  The "#pickup X" command already includes driving to the object, so when asked to grab or
   pick up an object, output "#pickup X" by itself and never output "#pilottoobject X" before it.
  To drop an object you are holding, output the string "#drop" without quotes.
  To perform a kick action, output the string "#kick" without quotes.
  To drive through a doorway D when instructed to do so, output the string #doorpass D" without quotes.
  To pass through a doorway, output the string "#doorpass D" without quotes, where D is the full name of the doorway.
  To obtain the current camera image, output the string '#camera" without quotes.
  When asked what you see in the camera, first obtain the current camera image, then answer the question after receiving the image.
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
        super().__init__(character_name="Celeste",
                         launch_cam_viewer=True,
                         launch_worldmap_viewer=True,
                         launch_particle_viewer=True,
                         launch_path_viewer=True)

    def picked_up_celeste(self):
        self.robot.gpt_note_for_later("You have been picked up.")

    def put_down_celeste(self):
        self.stop_children()
        self.robot.gpt_note_for_later("You were picked up but have now been put down.")
        self.children['putdown'].start()

    def start(self):
        self.robot.openai_client.set_preamble(new_preamble)
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
                pat = re.compile(re.sub(r'[\s_-]+', '', spec), re.IGNORECASE)
                candidates = [o for o in self.robot.world_map.objects.values()
                              if pat.search(o.name) and o.is_valid]
                if candidates:
                    x, y = self.robot.pose.x, self.robot.pose.y
                    obj = min(candidates,
                              key=lambda o: (o.pose.x - x)**2 + (o.pose.y - y)**2)
                else:
                    obj = None
            return obj

    class CheckResponse(StateNode):
        def start(self, event):
            super().start(event)
            response_string = event.response
            lines = list(filter(lambda x: len(x)>0, response_string.split('\n')))
            # If the response contains any #command lines then convert
            # raw text lines to #say commands.
            if any((line.startswith('#') for line in lines)):
                commands = [line if line.startswith('#') else ('#say ' + line) for line in lines]
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
            super().start(None)

    class CmdFailed(AskGPT):
        def __init__(self, query_template, filler_fn=lambda : ()):
            super().__init__()
            self.query_template = query_template
            self.filler_fn = filler_fn
            
        def start(self,event=None):
            self.query_text = self.query_template % self.filler_fn()
            super().start()


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
                if obj is not None and (obj.is_visible or isinstance(obj, ArucoMarkerObj)):
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
            
            # Code generated by genfsm on Sat Aug 29 03:20:25 2026:
            
            nearcheck = self.NearCheck() .set_name("nearcheck") .set_parent(self)
            backup = Forward(-100) .set_name("backup") .set_parent(self)
            prior = self.PriorTurn() .set_name("prior") .set_parent(self)
            check = self.CheckVisible() .set_name("check") .set_parent(self)
            parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
            turn1 = Turn(self.TURN_STEP_DEG) .set_name("turn1") .set_parent(self)
            look = StateNode() .set_name("look") .set_parent(self)
            recenter = self.Recenter() .set_name("recenter") .set_parent(self)
            settle = StateNode() .set_name("settle") .set_parent(self)
            announce = self.AnnounceFound() .set_name("announce") .set_parent(self)
            parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
            parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
            
            successtrans1 = SuccessTrans() .set_name("successtrans1")
            successtrans1 .add_sources(nearcheck) .add_destinations(backup)
            
            failuretrans1 = FailureTrans() .set_name("failuretrans1")
            failuretrans1 .add_sources(nearcheck) .add_destinations(prior)
            
            completiontrans1 = CompletionTrans() .set_name("completiontrans1")
            completiontrans1 .add_sources(backup) .add_destinations(prior)
            
            failuretrans2 = FailureTrans() .set_name("failuretrans2")
            failuretrans2 .add_sources(backup) .add_destinations(prior)
            
            completiontrans2 = CompletionTrans() .set_name("completiontrans2")
            completiontrans2 .add_sources(prior) .add_destinations(look)
            
            failuretrans3 = FailureTrans() .set_name("failuretrans3")
            failuretrans3 .add_sources(prior) .add_destinations(check)
            
            successtrans2 = SuccessTrans() .set_name("successtrans2")
            successtrans2 .add_sources(check) .add_destinations(recenter)
            
            failuretrans4 = FailureTrans() .set_name("failuretrans4")
            failuretrans4 .add_sources(check) .add_destinations(parentfails1)
            
            completiontrans3 = CompletionTrans() .set_name("completiontrans3")
            completiontrans3 .add_sources(check) .add_destinations(turn1)
            
            completiontrans4 = CompletionTrans() .set_name("completiontrans4")
            completiontrans4 .add_sources(turn1) .add_destinations(look)
            
            timertrans1 = TimerTrans(self.PAUSE_SECS) .set_name("timertrans1")
            timertrans1 .add_sources(look) .add_destinations(check)
            
            completiontrans5 = CompletionTrans() .set_name("completiontrans5")
            completiontrans5 .add_sources(recenter) .add_destinations(settle)
            
            failuretrans5 = FailureTrans() .set_name("failuretrans5")
            failuretrans5 .add_sources(recenter) .add_destinations(settle)
            
            timertrans2 = TimerTrans(self.SETTLE_SECS) .set_name("timertrans2")
            timertrans2 .add_sources(settle) .add_destinations(announce)
            
            completiontrans6 = CompletionTrans() .set_name("completiontrans6")
            completiontrans6 .add_sources(announce) .add_destinations(parentcompletes1)
            
            failuretrans6 = FailureTrans() .set_name("failuretrans6")
            failuretrans6 .add_sources(announce) .add_destinations(parentcompletes2)
            
            return self

    class CmdDrop(StateNode):
      def start(self,event):
          print(event.data)
          super().start(event)
      def setup(self):
          #           drop: Drop()
          #           drop =F=> ParentCompletes()
          #           drop =C=> ParentCompletes()
          
          # Code generated by genfsm on Sat Aug 29 03:20:25 2026:
          
          drop = Drop() .set_name("drop") .set_parent(self)
          parentcompletes3 = ParentCompletes() .set_name("parentcompletes3") .set_parent(self)
          parentcompletes4 = ParentCompletes() .set_name("parentcompletes4") .set_parent(self)
          
          failuretrans7 = FailureTrans() .set_name("failuretrans7")
          failuretrans7 .add_sources(drop) .add_destinations(parentcompletes3)
          
          completiontrans7 = CompletionTrans() .set_name("completiontrans7")
          completiontrans7 .add_sources(drop) .add_destinations(parentcompletes4)
          
          return self

    class CmdKick(MediumKick):  
      def start(self,event):
          print(event.data)
          super().start(event)

    class CmdSendCamera(SendGPTCamera):
        def start(self,event):
            print(event.data)
            super().start(event)

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
            
            # Code generated by genfsm on Sat Aug 29 03:20:25 2026:
            
            dispatch = self.SendAction() .set_name("dispatch") .set_parent(self)
            acthappy1 = ActHappy() .set_name("acthappy1") .set_parent(self)
            actsad1 = ActSad() .set_name("actsad1") .set_parent(self)
            actsilly1 = ActSilly() .set_name("actsilly1") .set_parent(self)
            actangry1 = ActAngry() .set_name("actangry1") .set_parent(self)
            actexcited1 = ActExcited() .set_name("actexcited1") .set_parent(self)
            complete = ParentCompletes() .set_name("complete") .set_parent(self)
            
            datatrans1 = DataTrans('happy') .set_name("datatrans1")
            datatrans1 .add_sources(dispatch) .add_destinations(acthappy1)
            
            completiontrans8 = CompletionTrans() .set_name("completiontrans8")
            completiontrans8 .add_sources(acthappy1) .add_destinations(complete)
            
            datatrans2 = DataTrans('sad') .set_name("datatrans2")
            datatrans2 .add_sources(dispatch) .add_destinations(actsad1)
            
            completiontrans9 = CompletionTrans() .set_name("completiontrans9")
            completiontrans9 .add_sources(actsad1) .add_destinations(complete)
            
            datatrans3 = DataTrans('silly') .set_name("datatrans3")
            datatrans3 .add_sources(dispatch) .add_destinations(actsilly1)
            
            completiontrans10 = CompletionTrans() .set_name("completiontrans10")
            completiontrans10 .add_sources(actsilly1) .add_destinations(complete)
            
            datatrans4 = DataTrans('angry') .set_name("datatrans4")
            datatrans4 .add_sources(dispatch) .add_destinations(actangry1)
            
            completiontrans11 = CompletionTrans() .set_name("completiontrans11")
            completiontrans11 .add_sources(actangry1) .add_destinations(complete)
            
            datatrans5 = DataTrans('excited') .set_name("datatrans5")
            datatrans5 .add_sources(dispatch) .add_destinations(actexcited1)
            
            completiontrans12 = CompletionTrans() .set_name("completiontrans12")
            completiontrans12 .add_sources(actexcited1) .add_destinations(complete)
            
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
        #         loop: StateNode() =Hear()=> AskGPT() =OpenAITrans()=> check
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
        #         dispatch =D(re.compile('#doorpass '))=> doorpass
        #         dispatch =D(re.compile('#pickup '))=> pickup
        #         dispatch =D(re.compile('#drop$'))=> self.CmdDrop() =CNext=> dispatch
        #         dispatch =D(re.compile('#kick$'))=> self.CmdKick() =CNext=> dispatch
        #         dispatch =D(re.compile('#glow '))=> self.CmdGlow() =CNext=> dispatch
        #         dispatch =D(re.compile('#flash '))=> self.CmdFlash() =CNext=> dispatch
        #         dispatch =D(re.compile('#emoji '))=> self.CmdEmoji() =CNext=> dispatch
        #         dispatch =D(re.compile('#act '))=> self.CmdAct() =CNext=> dispatch
        #         dispatch =D(re.compile('#playnotes '))=> self.CmdPlayNotes() =CNext=> dispatch
        #         dispatch =D(re.compile('#camera$'))=> self.CmdSendCamera() =C=>
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
        #         pilottoobject =CNext=> dispatch
        #         pilottoobject =PILOT(GoalUnreachable)=>
        #             self.CmdFailed("The object %s is not reachable due to obstructions", lambda : pilottoobject.object_spec) =OpenAITrans()=> check
        #         pilottoobject =F=>
        #             self.CmdFailed("The name '%s' is not a valid object name.", lambda : pilottoobject.object_spec) =OpenAITrans()=> check
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
        
        # Code generated by genfsm on Sat Aug 29 03:20:25 2026:
        
        print1 = Print(f"Celeste version {CELESTE_VERSION}") .set_name("print1") .set_parent(self)
        tagdetection1 = TagDetection(True) .set_name("tagdetection1") .set_parent(self)
        say1 = Say("Talk to me") .set_name("say1") .set_parent(self)
        putdown = Say(["I'm good", "Okay then", "I'm back", "Now then"]) .set_name("putdown") .set_parent(self)
        loop = StateNode() .set_name("loop") .set_parent(self)
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
        cmddrop1 = self.CmdDrop() .set_name("cmddrop1") .set_parent(self)
        cmdkick1 = self.CmdKick() .set_name("cmdkick1") .set_parent(self)
        cmdglow1 = self.CmdGlow() .set_name("cmdglow1") .set_parent(self)
        cmdflash1 = self.CmdFlash() .set_name("cmdflash1") .set_parent(self)
        cmdemoji1 = self.CmdEmoji() .set_name("cmdemoji1") .set_parent(self)
        cmdact1 = self.CmdAct() .set_name("cmdact1") .set_parent(self)
        cmdplaynotes1 = self.CmdPlayNotes() .set_name("cmdplaynotes1") .set_parent(self)
        cmdsendcamera1 = self.CmdSendCamera() .set_name("cmdsendcamera1") .set_parent(self)
        askgpt2 = AskGPT("Please respond to the query using the camera image.") .set_name("askgpt2") .set_parent(self)
        print2 = Print(prefix='Unrecognized #-command: ') .set_name("print2") .set_parent(self)
        say = self.CmdSay() .set_name("say") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        script = self.CmdScript() .set_name("script") .set_parent(self)
        say2 = Say("Script loaded") .set_name("say2") .set_parent(self)
        cmdfailed1 = self.CmdFailed("I am unable to load that file") .set_name("cmdfailed1") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        turntoward = self.CmdTurnToward() .set_name("turntoward") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        pilottoobject = self.CmdPilotToObject() .set_name("pilottoobject") .set_parent(self)
        cmdfailed2 = self.CmdFailed("The object %s is not reachable due to obstructions", lambda : pilottoobject.object_spec) .set_name("cmdfailed2") .set_parent(self)
        cmdfailed3 = self.CmdFailed("The name '%s' is not a valid object name.", lambda : pilottoobject.object_spec) .set_name("cmdfailed3") .set_parent(self)
        doorpass = self.CmdDoorPass() .set_name("doorpass") .set_parent(self)
        cmdfailed4 = self.CmdFailed("Doorpass failed for '%s'", lambda : doorpass.door_spec) .set_name("cmdfailed4") .set_parent(self)
        pickup = self.CmdPickup() .set_name("pickup") .set_parent(self)
        statenode4 = StateNode() .set_name("statenode4") .set_parent(self)
        search = self.CmdSearch() .set_name("search") .set_parent(self)
        cmdfailed5 = self.CmdFailed(self.SEARCH_FAIL_MSG, lambda : search.object_spec) .set_name("cmdfailed5") .set_parent(self)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(print1) .add_destinations(tagdetection1)
        
        timertrans3 = TimerTrans(2) .set_name("timertrans3")
        timertrans3 .add_sources(tagdetection1) .add_destinations(say1)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(say1) .add_destinations(loop)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(putdown) .add_destinations(loop)
        
        heartrans1 = HearTrans() .set_name("heartrans1")
        heartrans1 .add_sources(loop) .add_destinations(askgpt1)
        
        openaitrans1 = OpenAITrans() .set_name("openaitrans1")
        openaitrans1 .add_sources(askgpt1) .add_destinations(check)
        
        datatrans6 = DataTrans(list) .set_name("datatrans6")
        datatrans6 .add_sources(check) .add_destinations(dispatch)
        
        datatrans7 = DataTrans(str) .set_name("datatrans7")
        datatrans7 .add_sources(check) .add_destinations(speakresponse1)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(speakresponse1) .add_destinations(loop)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(reset_fsm) .add_destinations(loop)
        
        datatrans8 = DataTrans(re.compile('#hang$')) .set_name("datatrans8")
        datatrans8 .add_sources(dispatch) .add_destinations(cmdhang1)
        
        datatrans9 = DataTrans(re.compile('#script ')) .set_name("datatrans9")
        datatrans9 .add_sources(dispatch) .add_destinations(script)
        
        datatrans10 = DataTrans(re.compile('#say ')) .set_name("datatrans10")
        datatrans10 .add_sources(dispatch) .add_destinations(say)
        
        datatrans11 = DataTrans(re.compile('#forward ')) .set_name("datatrans11")
        datatrans11 .add_sources(dispatch) .add_destinations(cmdforward1)
        
        cnexttrans1 = CNextTrans() .set_name("cnexttrans1")
        cnexttrans1 .add_sources(cmdforward1) .add_destinations(dispatch)
        
        datatrans12 = DataTrans(re.compile('#sideways ')) .set_name("datatrans12")
        datatrans12 .add_sources(dispatch) .add_destinations(cmdsideways1)
        
        cnexttrans2 = CNextTrans() .set_name("cnexttrans2")
        cnexttrans2 .add_sources(cmdsideways1) .add_destinations(dispatch)
        
        datatrans13 = DataTrans(re.compile('#turn ')) .set_name("datatrans13")
        datatrans13 .add_sources(dispatch) .add_destinations(cmdturn1)
        
        cnexttrans3 = CNextTrans() .set_name("cnexttrans3")
        cnexttrans3 .add_sources(cmdturn1) .add_destinations(dispatch)
        
        datatrans14 = DataTrans(re.compile('#turntoward ')) .set_name("datatrans14")
        datatrans14 .add_sources(dispatch) .add_destinations(turntoward)
        
        datatrans15 = DataTrans(re.compile('#search ')) .set_name("datatrans15")
        datatrans15 .add_sources(dispatch) .add_destinations(search)
        
        datatrans16 = DataTrans(re.compile('#pilottoobject ')) .set_name("datatrans16")
        datatrans16 .add_sources(dispatch) .add_destinations(pilottoobject)
        
        datatrans17 = DataTrans(re.compile('#doorpass ')) .set_name("datatrans17")
        datatrans17 .add_sources(dispatch) .add_destinations(doorpass)
        
        datatrans18 = DataTrans(re.compile('#pickup ')) .set_name("datatrans18")
        datatrans18 .add_sources(dispatch) .add_destinations(pickup)
        
        datatrans19 = DataTrans(re.compile('#drop$')) .set_name("datatrans19")
        datatrans19 .add_sources(dispatch) .add_destinations(cmddrop1)
        
        cnexttrans4 = CNextTrans() .set_name("cnexttrans4")
        cnexttrans4 .add_sources(cmddrop1) .add_destinations(dispatch)
        
        datatrans20 = DataTrans(re.compile('#kick$')) .set_name("datatrans20")
        datatrans20 .add_sources(dispatch) .add_destinations(cmdkick1)
        
        cnexttrans5 = CNextTrans() .set_name("cnexttrans5")
        cnexttrans5 .add_sources(cmdkick1) .add_destinations(dispatch)
        
        datatrans21 = DataTrans(re.compile('#glow ')) .set_name("datatrans21")
        datatrans21 .add_sources(dispatch) .add_destinations(cmdglow1)
        
        cnexttrans6 = CNextTrans() .set_name("cnexttrans6")
        cnexttrans6 .add_sources(cmdglow1) .add_destinations(dispatch)
        
        datatrans22 = DataTrans(re.compile('#flash ')) .set_name("datatrans22")
        datatrans22 .add_sources(dispatch) .add_destinations(cmdflash1)
        
        cnexttrans7 = CNextTrans() .set_name("cnexttrans7")
        cnexttrans7 .add_sources(cmdflash1) .add_destinations(dispatch)
        
        datatrans23 = DataTrans(re.compile('#emoji ')) .set_name("datatrans23")
        datatrans23 .add_sources(dispatch) .add_destinations(cmdemoji1)
        
        cnexttrans8 = CNextTrans() .set_name("cnexttrans8")
        cnexttrans8 .add_sources(cmdemoji1) .add_destinations(dispatch)
        
        datatrans24 = DataTrans(re.compile('#act ')) .set_name("datatrans24")
        datatrans24 .add_sources(dispatch) .add_destinations(cmdact1)
        
        cnexttrans9 = CNextTrans() .set_name("cnexttrans9")
        cnexttrans9 .add_sources(cmdact1) .add_destinations(dispatch)
        
        datatrans25 = DataTrans(re.compile('#playnotes ')) .set_name("datatrans25")
        datatrans25 .add_sources(dispatch) .add_destinations(cmdplaynotes1)
        
        cnexttrans10 = CNextTrans() .set_name("cnexttrans10")
        cnexttrans10 .add_sources(cmdplaynotes1) .add_destinations(dispatch)
        
        datatrans26 = DataTrans(re.compile('#camera$')) .set_name("datatrans26")
        datatrans26 .add_sources(dispatch) .add_destinations(cmdsendcamera1)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(cmdsendcamera1) .add_destinations(askgpt2)
        
        openaitrans2 = OpenAITrans() .set_name("openaitrans2")
        openaitrans2 .add_sources(askgpt2) .add_destinations(check)
        
        datatrans27 = DataTrans() .set_name("datatrans27")
        datatrans27 .add_sources(dispatch) .add_destinations(print2)
        
        nexttrans1 = NextTrans() .set_name("nexttrans1")
        nexttrans1 .add_sources(print2) .add_destinations(dispatch)
        
        completiontrans18 = CompletionTrans() .set_name("completiontrans18")
        completiontrans18 .add_sources(dispatch) .add_destinations(loop)
        
        cnexttrans11 = CNextTrans() .set_name("cnexttrans11")
        cnexttrans11 .add_sources(say) .add_destinations(dispatch)
        
        timertrans4 = TimerTrans(5) .set_name("timertrans4")
        timertrans4 .add_sources(say) .add_destinations(statenode1)
        
        nexttrans2 = NextTrans() .set_name("nexttrans2")
        nexttrans2 .add_sources(statenode1) .add_destinations(dispatch)
        
        completiontrans19 = CompletionTrans() .set_name("completiontrans19")
        completiontrans19 .add_sources(script) .add_destinations(say2)
        
        cnexttrans12 = CNextTrans() .set_name("cnexttrans12")
        cnexttrans12 .add_sources(say2) .add_destinations(dispatch)
        
        failuretrans8 = FailureTrans() .set_name("failuretrans8")
        failuretrans8 .add_sources(script) .add_destinations(cmdfailed1)
        
        openaitrans3 = OpenAITrans() .set_name("openaitrans3")
        openaitrans3 .add_sources(cmdfailed1) .add_destinations(check)
        
        timertrans5 = TimerTrans(5) .set_name("timertrans5")
        timertrans5 .add_sources(script) .add_destinations(statenode2)
        
        nexttrans3 = NextTrans() .set_name("nexttrans3")
        nexttrans3 .add_sources(statenode2) .add_destinations(dispatch)
        
        cnexttrans13 = CNextTrans() .set_name("cnexttrans13")
        cnexttrans13 .add_sources(turntoward) .add_destinations(dispatch)
        
        failuretrans9 = FailureTrans() .set_name("failuretrans9")
        failuretrans9 .add_sources(turntoward) .add_destinations(statenode3)
        
        nexttrans4 = NextTrans() .set_name("nexttrans4")
        nexttrans4 .add_sources(statenode3) .add_destinations(dispatch)
        
        cnexttrans14 = CNextTrans() .set_name("cnexttrans14")
        cnexttrans14 .add_sources(pilottoobject) .add_destinations(dispatch)
        
        pilottrans1 = PilotTrans(GoalUnreachable) .set_name("pilottrans1")
        pilottrans1 .add_sources(pilottoobject) .add_destinations(cmdfailed2)
        
        openaitrans4 = OpenAITrans() .set_name("openaitrans4")
        openaitrans4 .add_sources(cmdfailed2) .add_destinations(check)
        
        failuretrans10 = FailureTrans() .set_name("failuretrans10")
        failuretrans10 .add_sources(pilottoobject) .add_destinations(cmdfailed3)
        
        openaitrans5 = OpenAITrans() .set_name("openaitrans5")
        openaitrans5 .add_sources(cmdfailed3) .add_destinations(check)
        
        cnexttrans15 = CNextTrans() .set_name("cnexttrans15")
        cnexttrans15 .add_sources(doorpass) .add_destinations(dispatch)
        
        failuretrans11 = FailureTrans() .set_name("failuretrans11")
        failuretrans11 .add_sources(doorpass) .add_destinations(cmdfailed4)
        
        openaitrans6 = OpenAITrans() .set_name("openaitrans6")
        openaitrans6 .add_sources(cmdfailed4) .add_destinations(check)
        
        cnexttrans16 = CNextTrans() .set_name("cnexttrans16")
        cnexttrans16 .add_sources(pickup) .add_destinations(dispatch)
        
        failuretrans12 = FailureTrans() .set_name("failuretrans12")
        failuretrans12 .add_sources(pickup) .add_destinations(statenode4)
        
        nexttrans5 = NextTrans() .set_name("nexttrans5")
        nexttrans5 .add_sources(statenode4) .add_destinations(dispatch)
        
        cnexttrans17 = CNextTrans() .set_name("cnexttrans17")
        cnexttrans17 .add_sources(search) .add_destinations(dispatch)
        
        failuretrans13 = FailureTrans() .set_name("failuretrans13")
        failuretrans13 .add_sources(search) .add_destinations(cmdfailed5)
        
        openaitrans7 = OpenAITrans() .set_name("openaitrans7")
        openaitrans7 .add_sources(cmdfailed5) .add_destinations(check)
        
        return self
