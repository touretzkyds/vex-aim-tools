class ActuatorLocked(Exception):
    pass

class Actuator():
    def __init__(self, robot, name, stop_fn = lambda : None):
        self.robot = robot
        self.name = name
        self.holder = None
        self.stop_fn = stop_fn

    def __repr__(self):
        return f"<Actuator {self.name}>"

    def status_update(self): pass

    def lock(self, node):
        if self.holder is None:
            self.holder = node
            return True
        else:
            raise ActuatorLocked(self)

    def unlock(self):
        self.holder = None

    def complete(self):
        print(f"{self.name} completes, holder {self.holder}")
        if self.holder:
            self.holder.complete()
        self.holder = None

class DriveActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'drive')

    def stop(self):
        self.robot.robot0.stop_drive()

    def status_update(self):
        if self.holder and not self.robot.robot0.is_moving():
            self.holder.complete()
            self.holder = None

class SoundActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'sound')

class KickActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'kick')

class LEDsActuator(Actuator):
    def __init__(self, robot):
        super().__init__(robot, 'leds')

    def stop(self):
        self.robot.robot0.clear_leds()

