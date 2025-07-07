from aim_fsm import *

emoji_list = ', '.join([key for (key,value) in vars(vex.EmojiType).items()
                        if isinstance(value, vex.EmojiType.EmojiType)])
print('el=', emoji_list)
new_preamble = """
  IDENTITY SECTION.
  You are an intelligent mobile robot named Celeste.
  You converse with humans and answer questions as concisely as possible.
  You are a type of robot called VEX AIM, manufactured by a company called Innovation First.
  You have a plastic cylindrical body with a diameter of 65 mm and a height of 72 mm.
  You have three omnidirectional wheels and a forward-facing camera.
  You have six color LEDs evenly spaced around your body.
  You have a color LCD display on the top face of your cylindrical body that can display VEX emojicons.
  The list of {len(emoji_list)} VEX emojicons you can display is: {emoji_list}. Please remember this list.

  BODY CONTROL SECTION.
  Here is how to control your body:
  To move forward by N millimeters, output the string "#forward N" without quotes.
  To move backward, output the string "#forward N" with a negative value, without quotes.
  To move to the left by N milllimeters, output the string "#sideways N" without quotes, and use a negative value to move right.
  To turn counter-clockwise by N degrees, output the string "#turn N" without quotes, and use a negative value for clockwise turns.
  To turn toward object X, output the string "#turntoward X" without quotes.
  To travel to object X, output the string #pilottoobject X" without quotes.
  To pick up object X, output the string "#pickup X" without quotes.
  To drop an object you are holding, output the string "#drop" without quotes.
  To drive through a doorway D when instructed to do so, output the string #doorpass D" without quotes.
  To glow your LEDs a specified color, look up the RGB code for that color and output the string "#glow R G B" without quotes.  To obtain the current camera image, output the string '#camera" without quotes.
  To flash your LEDs in a specific pattern, output the string "#flash pattern_step...", 
    where "pattern_step..." denotes a sequence of pattern_steps separated by spaces.
  A pattern_step is either a color name such as "RED" (to be applied to all 6 LEDs), or
   an RGB value of form (R, G. B), or
   a list of six color names such as "(RED, BLUE, RED, BLUE, GREEN, TRANSPARENT)".
   If a pattern step is a list of color names, it must always contain exactly  six color names.
  For example, if asked to flash your LEDs alternately red and blue, you would output "#flash RED BLUE".  Each of "RED" and "BLUE" is a pattern_step.
  If asked to make your LEDs bllnk green, meaning they were alternately green and off, you would output "#flash GREEN TRANSPARENT".
  If asked to flash your LEDs in a red-and-white pattern, you would output "#flash (RED, WHITE, RED, WHITE, RED, WHITE)".
  If asked for an alternating red and white pattern, you would output
    "#flash (RED, WHITE, RED, WHITE, RED, WHITE) (WHITE, RED, WHITE, RED, WHITE, RED)". Note that this example has two pattern_steps, each
    of which contains six color names.
  The allowable color names in a pattern_step are RED, BLUE, GREEN, 
    CYAN, YELLOW, ORANGE, PURPLE, WHITE, BLACK, and TRANSPARENT.  For all other colors, use the RGB code.
  Whenever you are asked to flash or blink your LEDs, use "#flash" and not "#glow".
  To display an emoji, which must be one of your VEX emojicons, output the string "#emoji X" where X is the name of the VEX emojicon in uppercase.
  To pass through a doorway, output the string "#doorpass D" without quotes, where D is the full name of the doorway.
  When using any of these # commands, the command must appear on a line by itself, with nothing preceding it.
  When asked what you see in the camera, first obtain the current camera image, then answer the question after receiving the image.

  MUSICAL NOTES SECTION.
  You can play musical notes ranging from C5 (middle C) to A8.
  For a sharp write C#5.
  You can only play one note at a time; you cannot play chords.
  To play a sequence of notes like C5, E5, G5, output "#playnotes C5 E5 G5".
  The symbol C5 denotes a quarter note.  An appended underscore  doubles the note's duration.
  So for a half note, write C5_.  For a whole note write C5__.
  An appended minus sign halves the duration.  For an eighth note write C5-.
  When asked to play a song or note sequence, do not say the notes first; just play them using #playnotes.

  PRONUNCIATION SECTION.
  Pronounce "AprilTag-1.a" as "April Tag 1-A", and similarly for any word of form "AprilTag-N.x".
  Pronounce "OrangeBarrel.a" as "Orange Barrel A", pronounce "BlueBarrel.b" as "Blue Barrel B", and similarly for other barrel designators.
  Prounounce "ArucoMarkerObj-2.a" as "Marker 2".
  Pronounce 'Wall-2.a' as "Wall 2".
  Pronounce "Doorway-2:0.a" as "Doorway 2".

  GENERAL ADVICE SECTION.
  Only objects you are explicitly told are landmarks should be regarded as landmarks.
  Remember to be concise in your answers.
  Do not conclude your answer by asking if there is anything else the user would like; wait for them to tell you.
  Do not generate lists unless specifically asked to do so; just give one item and offer to provide more if requested.
  Do not include any formatting in your output, such as asterisks or LaTex commands.  Use plain text only.
  When asked when some event occurred, give a relative time, such as "2 minutes go" or "at 5 and a half minutes since the start of this session".
  Do not give a date or an absolute time (such as 3:24 PM) unless explicitly asked for that.
"""

class Celeste(StateMachineProgram):

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
        def __init__(self, query_template, filler_fn):
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

    class CmdDrop(Drop):
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
                numeric_items = [int(i) for i in re.findall('\d+', pattern_step)]
                if len(numeric_items) == 3:
                    lights = [int(i) for i in numeric_items]
                else:
                    alpha_items = re.findall('\w+', pattern_step)
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
        

    class CmdPlayNotes(PlayNotes):
        def start(self, event):
            print(event.data)
            self.score = event.data[event.data.find(' ')+1:]
            super().start()


    class SpeakResponse(Say):
      def start(self,event):
        self.text = event.data
        super().start(event)
        
    def start(self):
        self.robot.openai_client.set_preamble(new_preamble)
        super().start()

    def setup(self):
        #         Say("Talk to me") =C=> loop
        # 
        #         loop: StateNode() =Hear()=> AskGPT() =OpenAITrans()=> check
        # 
        #         check: self.CheckResponse()
        #         check =D(list)=> dispatch
        #         check =D(str)=> self.SpeakResponse() =C=> loop
        # 
        #         dispatch: Iterate()
        #         dispatch =D(re.compile('#say '))=> self.CmdSay() =CNext=> dispatch
        #         dispatch =D(re.compile('#forward '))=> self.CmdForward() =CNext=> dispatch
        #         dispatch =D(re.compile('#sideways '))=> self.CmdSideways() =CNext=> dispatch
        #         dispatch =D(re.compile('#turn '))=> self.CmdTurn() =CNext=> dispatch
        #         dispatch =D(re.compile('#turntoward '))=> turntoward
        #         dispatch =D(re.compile('#pilottoobject '))=> pilottoobject
        #         dispatch =D(re.compile('#doorpass '))=> doorpass
        #         dispatch =D(re.compile('#pickup '))=> pickup
        #         dispatch =D(re.compile('#drop$'))=> self.CmdDrop() =CNext=> dispatch
        #         dispatch =D(re.compile('#glow '))=> self.CmdGlow() =CNext=> dispatch
        # 	dispatch =D(re.compile('#flash '))=> self.CmdFlash() =CNext=> dispatch
        # 	dispatch =D(re.compile('#emoji '))=> self.CmdEmoji() =CNext=> dispatch
        # 	dispatch =D(re.compile('#playnotes '))=> self.CmdPlayNotes() =CNext=> dispatch
        #         dispatch =D(re.compile('#camera$'))=> self.CmdSendCamera() =C=>
        #             AskGPT("Please respond to the query using the camera image.") =OpenAITrans()=> check
        #         dispatch =D()=> Print(prefix='Unrecognized #-command: ') =Next=> dispatch
        #         dispatch =C=> loop
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
        
        # Code generated by genfsm on Mon Jun  2 02:51:39 2025:
        
        say1 = Say("Talk to me") .set_name("say1") .set_parent(self)
        loop = StateNode() .set_name("loop") .set_parent(self)
        askgpt1 = AskGPT() .set_name("askgpt1") .set_parent(self)
        check = self.CheckResponse() .set_name("check") .set_parent(self)
        speakresponse1 = self.SpeakResponse() .set_name("speakresponse1") .set_parent(self)
        dispatch = Iterate() .set_name("dispatch") .set_parent(self)
        cmdsay1 = self.CmdSay() .set_name("cmdsay1") .set_parent(self)
        cmdforward1 = self.CmdForward() .set_name("cmdforward1") .set_parent(self)
        cmdsideways1 = self.CmdSideways() .set_name("cmdsideways1") .set_parent(self)
        cmdturn1 = self.CmdTurn() .set_name("cmdturn1") .set_parent(self)
        cmddrop1 = self.CmdDrop() .set_name("cmddrop1") .set_parent(self)
        cmdglow1 = self.CmdGlow() .set_name("cmdglow1") .set_parent(self)
        cmdflash1 = self.CmdFlash() .set_name("cmdflash1") .set_parent(self)
        cmdemoji1 = self.CmdEmoji() .set_name("cmdemoji1") .set_parent(self)
        cmdplaynotes1 = self.CmdPlayNotes() .set_name("cmdplaynotes1") .set_parent(self)
        cmdsendcamera1 = self.CmdSendCamera() .set_name("cmdsendcamera1") .set_parent(self)
        askgpt2 = AskGPT("Please respond to the query using the camera image.") .set_name("askgpt2") .set_parent(self)
        print1 = Print(prefix='Unrecognized #-command: ') .set_name("print1") .set_parent(self)
        turntoward = self.CmdTurnToward() .set_name("turntoward") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        pilottoobject = self.CmdPilotToObject() .set_name("pilottoobject") .set_parent(self)
        cmdfailed1 = self.CmdFailed("The object %s is not reachable due to obstructions", lambda : pilottoobject.object_spec) .set_name("cmdfailed1") .set_parent(self)
        cmdfailed2 = self.CmdFailed("The name '%s' is not a valid object name.", lambda : pilottoobject.object_spec) .set_name("cmdfailed2") .set_parent(self)
        doorpass = self.CmdDoorPass() .set_name("doorpass") .set_parent(self)
        cmdfailed3 = self.CmdFailed("Doorpass failed for '%s'", lambda : doorpass.door_spec) .set_name("cmdfailed3") .set_parent(self)
        pickup = self.CmdPickup() .set_name("pickup") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(say1) .add_destinations(loop)
        
        heartrans1 = HearTrans() .set_name("heartrans1")
        heartrans1 .add_sources(loop) .add_destinations(askgpt1)
        
        openaitrans1 = OpenAITrans() .set_name("openaitrans1")
        openaitrans1 .add_sources(askgpt1) .add_destinations(check)
        
        datatrans1 = DataTrans(list) .set_name("datatrans1")
        datatrans1 .add_sources(check) .add_destinations(dispatch)
        
        datatrans2 = DataTrans(str) .set_name("datatrans2")
        datatrans2 .add_sources(check) .add_destinations(speakresponse1)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(speakresponse1) .add_destinations(loop)
        
        datatrans3 = DataTrans(re.compile('#say ')) .set_name("datatrans3")
        datatrans3 .add_sources(dispatch) .add_destinations(cmdsay1)
        
        cnexttrans1 = CNextTrans() .set_name("cnexttrans1")
        cnexttrans1 .add_sources(cmdsay1) .add_destinations(dispatch)
        
        datatrans4 = DataTrans(re.compile('#forward ')) .set_name("datatrans4")
        datatrans4 .add_sources(dispatch) .add_destinations(cmdforward1)
        
        cnexttrans2 = CNextTrans() .set_name("cnexttrans2")
        cnexttrans2 .add_sources(cmdforward1) .add_destinations(dispatch)
        
        datatrans5 = DataTrans(re.compile('#sideways ')) .set_name("datatrans5")
        datatrans5 .add_sources(dispatch) .add_destinations(cmdsideways1)
        
        cnexttrans3 = CNextTrans() .set_name("cnexttrans3")
        cnexttrans3 .add_sources(cmdsideways1) .add_destinations(dispatch)
        
        datatrans6 = DataTrans(re.compile('#turn ')) .set_name("datatrans6")
        datatrans6 .add_sources(dispatch) .add_destinations(cmdturn1)
        
        cnexttrans4 = CNextTrans() .set_name("cnexttrans4")
        cnexttrans4 .add_sources(cmdturn1) .add_destinations(dispatch)
        
        datatrans7 = DataTrans(re.compile('#turntoward ')) .set_name("datatrans7")
        datatrans7 .add_sources(dispatch) .add_destinations(turntoward)
        
        datatrans8 = DataTrans(re.compile('#pilottoobject ')) .set_name("datatrans8")
        datatrans8 .add_sources(dispatch) .add_destinations(pilottoobject)
        
        datatrans9 = DataTrans(re.compile('#doorpass ')) .set_name("datatrans9")
        datatrans9 .add_sources(dispatch) .add_destinations(doorpass)
        
        datatrans10 = DataTrans(re.compile('#pickup ')) .set_name("datatrans10")
        datatrans10 .add_sources(dispatch) .add_destinations(pickup)
        
        datatrans11 = DataTrans(re.compile('#drop$')) .set_name("datatrans11")
        datatrans11 .add_sources(dispatch) .add_destinations(cmddrop1)
        
        cnexttrans5 = CNextTrans() .set_name("cnexttrans5")
        cnexttrans5 .add_sources(cmddrop1) .add_destinations(dispatch)
        
        datatrans12 = DataTrans(re.compile('#glow ')) .set_name("datatrans12")
        datatrans12 .add_sources(dispatch) .add_destinations(cmdglow1)
        
        cnexttrans6 = CNextTrans() .set_name("cnexttrans6")
        cnexttrans6 .add_sources(cmdglow1) .add_destinations(dispatch)
        
        datatrans13 = DataTrans(re.compile('#flash ')) .set_name("datatrans13")
        datatrans13 .add_sources(dispatch) .add_destinations(cmdflash1)
        
        cnexttrans7 = CNextTrans() .set_name("cnexttrans7")
        cnexttrans7 .add_sources(cmdflash1) .add_destinations(dispatch)
        
        datatrans14 = DataTrans(re.compile('#emoji ')) .set_name("datatrans14")
        datatrans14 .add_sources(dispatch) .add_destinations(cmdemoji1)
        
        cnexttrans8 = CNextTrans() .set_name("cnexttrans8")
        cnexttrans8 .add_sources(cmdemoji1) .add_destinations(dispatch)
        
        datatrans15 = DataTrans(re.compile('#playnotes ')) .set_name("datatrans15")
        datatrans15 .add_sources(dispatch) .add_destinations(cmdplaynotes1)
        
        cnexttrans9 = CNextTrans() .set_name("cnexttrans9")
        cnexttrans9 .add_sources(cmdplaynotes1) .add_destinations(dispatch)
        
        datatrans16 = DataTrans(re.compile('#camera$')) .set_name("datatrans16")
        datatrans16 .add_sources(dispatch) .add_destinations(cmdsendcamera1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(cmdsendcamera1) .add_destinations(askgpt2)
        
        openaitrans2 = OpenAITrans() .set_name("openaitrans2")
        openaitrans2 .add_sources(askgpt2) .add_destinations(check)
        
        datatrans17 = DataTrans() .set_name("datatrans17")
        datatrans17 .add_sources(dispatch) .add_destinations(print1)
        
        nexttrans1 = NextTrans() .set_name("nexttrans1")
        nexttrans1 .add_sources(print1) .add_destinations(dispatch)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(dispatch) .add_destinations(loop)
        
        cnexttrans10 = CNextTrans() .set_name("cnexttrans10")
        cnexttrans10 .add_sources(turntoward) .add_destinations(dispatch)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(turntoward) .add_destinations(statenode1)
        
        nexttrans2 = NextTrans() .set_name("nexttrans2")
        nexttrans2 .add_sources(statenode1) .add_destinations(dispatch)
        
        cnexttrans11 = CNextTrans() .set_name("cnexttrans11")
        cnexttrans11 .add_sources(pilottoobject) .add_destinations(dispatch)
        
        pilottrans1 = PilotTrans(GoalUnreachable) .set_name("pilottrans1")
        pilottrans1 .add_sources(pilottoobject) .add_destinations(cmdfailed1)
        
        openaitrans3 = OpenAITrans() .set_name("openaitrans3")
        openaitrans3 .add_sources(cmdfailed1) .add_destinations(check)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(pilottoobject) .add_destinations(cmdfailed2)
        
        openaitrans4 = OpenAITrans() .set_name("openaitrans4")
        openaitrans4 .add_sources(cmdfailed2) .add_destinations(check)
        
        cnexttrans12 = CNextTrans() .set_name("cnexttrans12")
        cnexttrans12 .add_sources(doorpass) .add_destinations(dispatch)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(doorpass) .add_destinations(cmdfailed3)
        
        openaitrans5 = OpenAITrans() .set_name("openaitrans5")
        openaitrans5 .add_sources(cmdfailed3) .add_destinations(check)
        
        cnexttrans13 = CNextTrans() .set_name("cnexttrans13")
        cnexttrans13 .add_sources(pickup) .add_destinations(dispatch)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(pickup) .add_destinations(statenode2)
        
        nexttrans3 = NextTrans() .set_name("nexttrans3")
        nexttrans3 .add_sources(statenode2) .add_destinations(dispatch)
        
        return self
