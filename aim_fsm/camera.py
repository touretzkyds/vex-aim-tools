import numpy

class Camera():
    def __init__(self):
        self.resolution = (640, 480)
        self.focal_length = (400, 400)
        self.center = (320.0, 240.0) # should be adjusted for the actual camera
        self.fov_x = 70 # degrees
        self.fov_y = 50 # degrees
        self.distortion_matrix = numpy.array([0,0,0,0,0])
