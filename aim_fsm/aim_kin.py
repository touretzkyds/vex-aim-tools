from math import pi, tan

from .kine import *
from . import geometry
from .geometry import tprint, point, translation_part, rotation_part
from .rrt_shapes import *

# ================ Constants ================

camera_tilt = 18 # degrees downward (design spec)
camera_tilt = 25 # degrees downward (estimated)
camera_tilt = 24
camera_tilt = 25.4 # degrees downward (measured for AIM-5A888218)
camera_tilt = 23.7 # degrees downward (measured for AIM-32A57C18)

# Camera tilt Measurement procedure:
#  1. Abut ruler to front edge of robot; align edge of ruler with vertical crosshair.
#  2. Measure distance d to horizontal crosshair; measure on the ruler surface, not on the table.
#  3. Calculate camera_tilt = atan2(43.47, d + 3) * 180/pi


# ================================================================

class AIMKinematics(Kinematics):
    body_diameter = 60 # mm
    wheel_diameter = 24 # mm
    wheel_distance = 25 # mm from center
    robot_height = 72 # mm
    kicker_extension = 15 # mm

    camera_height = 43.47 # mm
    camera_from_origin = 27 # mm (approx distance from robot's center)

    def __init__(self,robot):
        base_frame = Joint('base',
                           description='Base frame: the root of the kinematic tree',
                           collision_model = Circle(geometry.point(),
                                                    radius = self.body_diameter / 2,
                                                    obstacle_id = 'robot'))

        # Use link instead of joint for world_frame
        world_frame = Joint('world', parent=base_frame, type='world', getter=self.get_world,
                            description='World origin in base frame coordinates',
                            qmin=None, qmax=None)

        kicker_frame = \
            Joint('kicker', parent=base_frame, type='prismatic',
                  description='The kicker',
                  d = 0, theta = 0, r = 31, alpha = 0,
                  qmin = 0,
                  qmax = self.kicker_extension,
                  #collision_model=Circle(geometry.point(), radius=10))
            )

        # camera dummy: located above base frame but oriented correctly.
        #
        # Two similar triangles: the smaller one is determined by
        # camera_height and camera_tilt; its apex is located at the
        # camera.  The larger triangle's apex is directly above the
        # base frame origin.
        y1 = self.camera_height
        x1 =  y1 / tan(camera_tilt*pi/180)
        x2 = x1 + self.camera_from_origin
        y2 = x2 * tan(camera_tilt*pi/180)
        camera_dummy = Joint('camera_dummy', parent=base_frame,
                             description='Camera dummy joint located above base frame',
                             d=y2, theta=-pi/2, alpha=-(90+camera_tilt)*pi/180)

        # camera frame: origin is at the actual camera; x axis points
        # right, y points down, z points forward
        r1 = (x1**2 + y1**2) ** 0.5
        r2 = (x2**2 + y2**2) ** 0.5
        camera_frame = Joint('camera', parent=camera_dummy,
                             description = 'Camera frame: x right, y down, z depth',
                             d = r2-r1)

        # display frame
        display_frame = Joint('display_frame', parent=base_frame,
                              description = 'LED display; x is down, y is right',
                              d = self.robot_height,
                              theta = pi)


        # wheel frames
        left_dummy = Joint('left_dummy', parent=base_frame,
                           description = 'Dummy joint for left wheel',
                           d = self.wheel_diameter/2, # axle height in mm
                           theta = -30 * pi/180,
                           alpha = -pi/2)

        left_wheel_frame = Joint('left_wheel', parent=left_dummy,
                                 description = 'Left wheel axle; y is up',
                                 d = self.wheel_distance,
                                 theta = pi)

        right_dummy = Joint('left_dummy', parent=base_frame,
                            description = 'Dummy joint for right wheel',
                            d = self.wheel_diameter/2, # axle height in mm
                            theta = 30 * pi/180,
                            alpha = pi/2)

        right_wheel_frame = Joint('right_wheel', parent=right_dummy,
                                  description = 'Right wheel axle; y is up',
                                  d = self.wheel_distance,
                                  theta = pi)

        rear_dummy = Joint('rear_dummy', parent=base_frame,
                            description = 'Dummy joint for rear wheel',
                            d = self.wheel_diameter/2, # axle height in mm
                            theta = -pi/2,
                            alpha = pi/2)

        rear_wheel_frame = Joint('rear_wheel', parent=rear_dummy,
                                 description = 'Rear wheel axle; y is up',
                                 d = self.wheel_distance)


        joints = [base_frame, world_frame, kicker_frame, camera_dummy, camera_frame,
                  display_frame, left_dummy, left_wheel_frame, right_dummy, right_wheel_frame,
                  rear_dummy, rear_wheel_frame]

        super().__init__(joints,robot)

    def get_world(self):
        pose = self.robot.pose
        return (pose.x, pose.y, pose.theta)

    def project_to_ground(self, cx, cy):
        "Converts camera coordinates to a ground point in the base frame."
        # Formula taken from Tekkotsu's projectToGround method
        camera_res = self.robot.camera.resolution
        half_camera_max = max(*camera_res) / 2
        # Convert to generalized coordinates in range [-1, 1]
        center = self.robot.camera.center
        gx = (cx - center[0]) / half_camera_max
        gy = (cy - center[1]) / half_camera_max
        # Generate a ray in the camera frame
        focal_length = self.robot.camera.focal_length
        rx = gx / (focal_length[0] / half_camera_max)
        ry = gy / (focal_length[1] / half_camera_max)
        ray = point(rx,ry,1)

        cam_to_base = self.robot.kine.joint_to_base('camera')
        offset = translation_part(cam_to_base)
        rot_ray = rotation_part(cam_to_base).dot(ray)
        dist = - offset[2,0]
        align = rot_ray[2,0]

        if abs(align) > 1e-5:
            s = dist / align
            hit = point(rot_ray[0,0]*s, rot_ray[1,0]*s, rot_ray[2,0]*s) + offset
        elif align * dist < 0:
            hit = point(-rot_ray[0,0], -rot_ray[1,0], -rot_ray[2,0], abs(align))
        else:
            hit = point(rot_ray[0,0], rot_ray[1,0], rot_ray[2,0], abs(align))
        return hit

import math

def world_to_image_coordinates(px, py, cx, cy, cz, f, v, r_x, r_y, d):
    # Translate the world coordinates by the camera position
    X_w = px - cx
    Y_w = py - cy
    Z_w = -cz
    
    # Rotate the translated coordinates by the tilt angle d (convert d to radians)
    d_rad = math.radians(d)
    X_c = X_w
    Y_c = Y_w * math.cos(d_rad) - Z_w * math.sin(d_rad)
    Z_c = Y_w * math.sin(d_rad) + Z_w * math.cos(d_rad)
    
    # Project the rotated coordinates onto the camera's image plane using pinhole camera model
    x_i = f * X_c / Z_c
    y_i = f * Y_c / Z_c
    
    # Calculate the width and height of the image in the camera's image plane
    width = 2 * f * math.tan(math.radians(v) / 2)
    height = width * r_y / r_x
    
    # Convert the normalized image coordinates to pixel coordinates
    u = r_x / 2 + x_i * r_x / width
    v = r_y / 2 - y_i * r_y / height
    
    return u, v
