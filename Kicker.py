from aim_fsm import *

class Kicker(StateMachineProgram):
  def setup(self):
      #     Kick() =C=> Print('Kicked') =T(0.5)=> Kick()
      
      # Code generated by genfsm on Thu Aug 22 19:29:54 2024:
      
      kick1 = Kick() .set_name("kick1") .set_parent(self)
      print1 = Print('Kicked') .set_name("print1") .set_parent(self)
      kick2 = Kick() .set_name("kick2") .set_parent(self)
      
      completiontrans1 = CompletionTrans() .set_name("completiontrans1")
      completiontrans1 .add_sources(kick1) .add_destinations(print1)
      
      timertrans1 = TimerTrans(0.5) .set_name("timertrans1")
      timertrans1 .add_sources(print1) .add_destinations(kick2)
      
      return self

