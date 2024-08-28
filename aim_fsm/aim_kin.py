from math import pi, tan

from .kine import *
from . import geometry
from .geometry import tprint, point, translation_part, rotation_part
#from .rrt_shapes import *

# ================ Constants ================

body_diameter = 57 # mm
kicker_extension = 15 # mm
camera_angle = 18 # degrees
camera_height = 43.47 # mm
camera_from_origin = 27 # mm (approx)

# ================================================================

class AIMKinematics(Kinematics):
    def __init__(self,robot):
        base_frame = Joint('base',
                           description='Base frame: the root of the kinematic tree')

        # Use link instead of joint for world_frame
        world_frame = Joint('world', parent=base_frame, type='world', getter=self.get_world,
                            description='World origin in base frame coordinates',
                            qmin=None, qmax=None)

        kicker_frame = \
            Joint('kicker', parent=base_frame, type='prismatic',
                  description='The kicker',
                  d = 0, theta = 0, r = 31, alpha = 0,
                  qmin = 0,
                  qmax = kicker_extension,
                  #collision_model=Circle(geometry.point(), radius=10))
            )

        # camera dummy: located above base frame but oriented correctly.
        #
        # Two similar triangles: the smaller one is determined by
        # camera_height and camera_angle; its apex is located at the
        # camera.  The larger triangle's apex is directly above the
        # base frame origin.
        y1 = camera_height
        x1 =  y1 / tan(camera_angle*pi/180)
        x2 = x1 + camera_from_origin
        y2 = x2 * tan(camera_angle*pi/180)
        camera_dummy = Joint('camera_dummy', parent=base_frame,
                             description='Camera dummy joint located above base frame',
                             d=y2, theta=-pi/2, alpha=-(90+camera_angle)*pi/180)

        # camera frame: x axis points right, y points down, z points forward
        r1 = (x1**2 + y1**2) ** 0.5
        r2 = (x2**2 + y2**2) ** 0.5
        camera_frame = Joint('camera', parent=camera_dummy,
                             description = 'Camera frame: x right, y down, z depth',
                             d = r2-r1)

        joints = [base_frame, world_frame, kicker_frame, camera_dummy, camera_frame]

        super().__init__(joints,robot)

    def get_world(self):
        return point(0,0,0) # *** FIX THIS SHOULD BE NEGATIVE OF ROBOT POSITION ***

    def project_to_ground(self,cx,cy):
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
