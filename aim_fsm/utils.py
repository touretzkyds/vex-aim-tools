import numpy as np
from math import pi, nan, sin, cos, atan2

from .geometry import wrap_angle, Quaternion

class Pose():
    def __init__(self, x=0, y=0, z=0, theta=None, quaternion = None, origin_id=-1):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        if quaternion is not None:
            self.quaternion = quaternion
        elif theta is not None:
            self.quaternion = Quaternion(angle_z = theta)
        else:
            self.quaternion = Quaternion(angle_z = 0)
        self.origin_id = origin_id

    def __repr__(self):
        theta = f' theta={self.theta*180/pi : .1f} deg.' if self.theta else ''
        return f'<Pose x={self.x:.1f} y={self.y:.1f} z={self.z:.1f}{theta} origin_id={self.origin_id}>'

    def __sub__(self, other):
        angdiff = wrap_angle(self.theta - other.theta) if self.theta is not None and other.theta is not None else None
        return Pose(self.x - other.x,
                    self.y - other.y,
                    self.z - other.z,
                    angdiff)
    
    def is_comparable(self, other):
        return self.origin_id == other.origin_id
    

class PoseEstimate(Pose):
    def __init__(self, x=0, y=0, z=0, theta=None, quaternion=None):
        if isinstance(x, Pose):
            p = x
            x = p.x
            y = p.y
            z = p.z
            theta = p.theta
            quaternion = p.quaternion
        super().__init__(x, y, z, theta, quaternion)

    def update(self, new_pose, dummy=None):
        linear_tolerance = 5
        angular_tolerance = pi/36 # 5 degrees
        object_moved = \
            (abs(self.x-new_pose.x) > linear_tolerance) or \
            (abs(self.y-new_pose.y) > linear_tolerance) or \
            (abs(self.y-new_pose.y) > linear_tolerance) or \
            (self.theta is not None and abs(wrap_angle(self.theta-new_pose.theta)) > angular_tolerance)
        if object_moved:
            self.x = new_pose.x
            self.y = new_pose.y
            self.z = new_pose.z
            self.theta = new_pose.theta
        else:
            weight = 0.1
            self.x = self.x*(1-weight) + new_pose.x*weight
            self.y = self.y*(1-weight) + new_pose.y*weight
            self.z = self.z*(1-weight) + new_pose.z*weight
            if self.theta is not None:
                weighted_sine = sin(self.theta)*(1-weight) + sin(new_pose.theta)*weight
                weighted_cosine = cos(self.theta)*(1-weight) + cos(new_pose.theta)*weight
                self.theta = atan2(weighted_sine, weighted_cosine)
            
    def __repr__(self):
        theta = f' theta={self.theta*180/pi : .1f} deg.' if self.theta else ''
        return f'<PoseEstimate x={self.x:.1f} y={self.y:.1f} z={self.z:.1f}{theta} origin_id={self.origin_id}>'
