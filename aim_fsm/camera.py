import numpy

class Camera():
    def __init__(self):
        self.resolution = (640, 480)
        self.focal_length = (300, 300)
        self.center = (320.0, 240.0) # should be adjusted for the actual robot's camera
        self.fov_x = 70 # degrees (est.)
        self.fov_y = 50 # degrees (est.)
        self.distortion_matrix = numpy.array([0,0,0,0,0])
