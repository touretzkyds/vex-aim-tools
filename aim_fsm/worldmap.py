class WorldMap():

    def __init__(self,robot):
        self.robot = robot
        self.objects = dict()
        self.shared_objects = dict()
        self.aruco = None

    def clear(self):
        self.objects.clear()
        #self.robot.world.particle_filter.clear_landmarks()

