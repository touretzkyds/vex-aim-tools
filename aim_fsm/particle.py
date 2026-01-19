"""
Particle filter localization.
"""
import inspect

from .utils import *
from . import aim_kin
import math
import random
import numpy as np
from math import pi, sqrt, sin, cos, atan2, exp
from .geometry import wrap_angle, wrap_selected_angles
from .aruco import ArucoMarker
from .worldmap import WorldObject, ArucoMarkerObj, WallObj

wall_marker_dict = dict()

class Particle():
    def __init__(self, index=-1):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.log_weight = 0
        self.weight = 1
        self.index = index

    def __repr__(self):
        return '<Particle %d: (%.2f, %.2f) %.1f deg. log_wt=%f>' % \
               (self.index, self.x, self.y, self.theta*80/pi, self.log_weight)
    
#================ Particle Initializers ================

#to create a particle filter initialisation
class ParticleInitializer():
    def __init__(self):
        self.pf = None   # must be filled in after creation

#random initialisation of particles within a certain radius
class RandomWithinRadius(ParticleInitializer):
    """ Normally distribute particles within a radius, with random heading. """
    def __init__(self,radius=200):
        super().__init__()
        self.radius = radius

    def initialize(self, robot):
        for p in self.pf.particles:  #pf.particles - list of particles
            qangle = random.random()*2*pi
            r = random.gauss(0, self.radius/2) + self.radius/1.5
            p.x = r * cos(qangle)
            p.y = r * sin(qangle)
            p.theta = random.random()*2*pi
            p.log_weight = 0.0
            p.weight = 1.0
        self.pf.pose = PoseEstimate(0, 0, 0, 0)
        self.pf.motion_model.old_pose = robot.pose

class RobotPosition(ParticleInitializer):
    """ Initialize all particles to the robot's current position or a constant;
    the motion model will jitter them. """
    def __init__(self, x=None, y=None, theta=None):
        super().__init__()
        self.x = x
        self.y = y
        self.theta = theta

    def initialize(self, robot):
        if self.x is None:
            x = robot.pose.x
            y = robot.pose.y
            theta = robot.pose.theta
        else:
            x = self.x
            y = self.y
            theta = self.theta
        for p in self.pf.particles:
            p.x = x
            p.y = y
            p.theta = theta
            p.log_weight = 0.0
            p.weight = 1.0
        self.pf.pose = PoseEstimate(x, y, 0, theta)
        self.pf.motion_model.old_pose = robot.pose


#================ Motion Model ================

class MotionModel():
    def __init__(self, robot):
        self.robot = robot
        self.last_pose = Pose(self.robot.robot0.get_y_position(),
                              - self.robot.robot0.get_x_position(),
                              0,
                              - self.robot.robot0.inertial.get_heading() * pi/180,
                              'from robot0'
        )

    def compute_robot_motion(self):
        # How much did we move since last evaluation?
        x = self.robot.robot0.get_y_position()
        y = - self.robot.robot0.get_x_position()
        theta = - wrap_angle(self.robot.robot0.inertial.get_heading() * pi/180)
        new_pose = Pose(x, y, 0, theta, 'from robot0')
        if not self.last_pose:  # robot was picked up
            self.last_pose = new_pose
        dx = x - self.last_pose.x
        dy = y - self.last_pose.y
        distance = sqrt(dx*dx + dy*dy)
        turn_angle = wrap_angle(theta - self.last_pose.theta)
        travel_direction = atan2(dy, dx)
        self.last_pose = new_pose
        return (distance, turn_angle, travel_direction, theta)

class DefaultVexMotionModel(MotionModel):
    DEFAULT_TRANSLATION_SIGMA = 0.01
    DEFAULT_ROTATION_SIGMA = 0.001
    def __init__(self, robot,
                 sigma_trans = DEFAULT_TRANSLATION_SIGMA,
                 sigma_rot = DEFAULT_ROTATION_SIGMA):
        super().__init__(robot)
        self.sigma_trans = sigma_trans
        self.sigma_rot = sigma_rot
        self.old_pose = robot.pose

    def move(self, particles):
        (distance, turn_angle, travel_direction, theta) = self.compute_robot_motion()
        rot_var = 0 if abs(turn_angle) < 0.001 else self.sigma_rot
        for p in particles:
            jdist = distance * (1 + random.gauss(0, self.sigma_trans))
            jdir = travel_direction + (p.theta - theta)  + random.gauss(0, self.sigma_rot)
            p.x += jdist * cos(jdir)
            p.y += jdist * sin(jdir)
            p.theta += random.gauss(turn_angle, rot_var)


#================ Sensor Model ================

class SensorModel():
    def __init__(self, robot, landmarks=None):
        self.robot = robot
        if landmarks is None:
            landmarks = dict()
        self.set_landmarks(landmarks)
        self.last_evaluate_pose = Pose(0, 0, 0, 0)

    def motion_since_last_evaluate(self):
        dx = self.robot.pose.x - self.last_evaluate_pose.x
        dy = self.robot.pose.y - self.last_evaluate_pose.y
        distance = (dx**2 + dy**2) ** 0.5
        turn_angle = wrap_angle(self.robot.pose.theta - self.last_evaluate_pose.theta)
        return (distance, turn_angle)

    def set_landmarks(self,landmarks):
        self.landmarks = landmarks

class ArucoDistanceSensorModel(SensorModel):
    """Sensor model using only landmark distances."""
    def __init__(self, robot, landmarks=None, distance_variance=100):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self, particles, force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be
        # worthwhile, unless forced.
        (distance, turn_angle) = self.motion_since_last_evaluate()
        if (not force) and (distance < 5) and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seen_marker_objects because vision is in another thread.
        seen_marker_objects = self.robot.aruco_detector.seen_marker_objects
        # Process each seen marker:
        for (id, marker) in seen_marker_objects.items():
            if marker.id_string in self.landmarks:
                sensor_dist = marker.camera_distance
                landmark_spec = self.landmarks[marker.id_string]
                lm_x = landmark_spec.x
                lm_y = landmark_spec.y
                for p in particles:
                    dx = lm_x - p.x
                    dy = lm_y - p.y
                    predicted_dist = sqrt(dx*dx + dy*dy)
                    error = sensor_dist - predicted_dist
                    p.log_weight -= (error*error)/self.distance_variance
        return True
    
class ArucoBearingSensorModel(SensorModel):
    """Sensor model using only landmark bearings."""
    def __init__(self, robot, landmarks=None, bearing_variance=0.1):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.bearing_variance = bearing_variance

    def evaluate(self, particles, force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (distance, turn_angle) = self.motion_since_last_evaluate()
        if not force and distance < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seen_marker_objects because vision is in another thread.
        seen_marker_objects = self.robot.aruco_detector.seen_marker_objects
        # Process each seen marker:
        for (id, marker) in seen_marker_objects.items():
            if marker.id_string in self.landmarks:
                camera_offset = np.array([0, 0, aim_kin.camera_from_origin])
                sensor_coords = marker.camera_coords + camera_offset
                sensor_bearing = atan2(sensor_coords[0], sensor_coords[2])
                landmark_spec = self.landmarks[marker.id_string] 
                lm_x = landmark_spec.x
                lm_y = landmark_spec.y
                for p in particles:
                    dx = lm_x - p.x
                    dy = lm_y - p.y
                    predicted_bearing = wrap_angle(atan2(dy,dx) - p.theta)
                    error = wrap_angle(sensor_bearing - predicted_bearing)
                    p.log_weight -= (error * error) / self.bearing_variance
        return True

class ArucoCombinedSensorModel(SensorModel):
    """Sensor model using combined distance and bearing information."""
    def __init__(self, robot, landmarks=None, distance_variance=200):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self, particles, force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Don't evaluate if robot is still moving; ArUco info will be bad.
        # if self.robot.is_moving:
        #     return False

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (distance, turn_angle) = self.motion_since_last_evaluate()
        if not force and distance < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seen_marker_objects because vision is in another thread.
        seen_marker_objects = self.robot.aruco_detector.seen_marker_objects
        # Process each seen marker:
        for (id, marker) in seen_marker_objects.items():
            if marker.id_string in self.landmarks:
                sensor_dist = marker.camera_distance
                camera_offset = np.array([0, 0, aim_kin.camera_from_origin])
                sensor_coords = marker.camera_coords + camera_offset
                sensor_bearing = atan2(sensor_coords[0], sensor_coords[2])
                landmark_spec = self.landmarks[marker.id_string]
                lm_x = landmark_spec.x
                lm_y = landmark_spec.y
                for p in particles:
                    # Use sensed bearing and distance to get particle's
                    # estimate of landmark position on the world map.
                    predicted_pos_x = p.x + sensor_dist * cos(p.theta + sensor_bearing)
                    predicted_pos_y = p.y + sensor_dist * sin(p.theta + sensor_bearing)
                    dx = lm_x - predicted_pos_x
                    dy = lm_y - predicted_pos_y
                    error_sq = dx*dx + dy*dy
                    p.log_weight -= error_sq / self.distance_variance
        return True
    

#================ Particle Filter ================

class ParticleFilter():
    # Particle filter state:
    LOCALIZED = 'localized'       # Normal
    LOCALIZING = 'localizing'     # Trying to use LMs to localize
    LOST = 'lost'                 # De-localized and no LMs in view

    def __init__(self, robot, num_particles=500,
                 initializer = RandomWithinRadius(),
                 randomizer = RandomWithinRadius(),
                 motion_model = "default",
                 sensor_model = "default",
                 particle_factory = Particle,
                 landmarks = None):
        if landmarks is None:
            landmarks = dict()   # make a fresh dict each time
        self.robot = robot
        self.num_particles = num_particles
        self.initializer = initializer
        self.initializer.pf = self
        self.randomizer = randomizer
        self.randomizer.pf = self

        if motion_model == "default":
            motion_model = DefaultVexMotionModel(robot)
        self.motion_model = motion_model
        self.motion_model.pf = self

        if sensor_model == "default":
            self.sensor_model = ArucoCombinedSensorModel(robot)
        elif isinstance(sensor_model, SensorModel):
            self.sensor_model = sensor_model
        elif callable(sensor_model):
            self.sensor_model = sensor_model(robot)
        elif inspect.isclass(sensor_model) and issubclass(sensor_model, SensorModel):
            self.sensor_model = sensor_model(robot)
        else:
            self.sensor_model = None
        if self.sensor_model:
            self.sensor_model.set_landmarks(landmarks)
            self.sensor_model.pf = self

        self.particle_factory = particle_factory
        self.particles = [particle_factory(i) for i in range(num_particles)]
        self.best_particle = self.particles[0]
        self.min_log_weight = -300  # prevent floating point underflow in exp()
        self.initializer.initialize(robot)
        self.exp_weights = np.empty(self.num_particles)
        self.cdf = np.empty(self.num_particles)
        self.variance = (np.array([[0,0],[0,0]]), 0.)
        self.new_indices = [0] * num_particles # lists are faster than arrays here
        self.new_x = [0.0] * num_particles # lists are faster than arrays here
        self.new_y = [0.0] * num_particles # lists are faster than arrays here`
        self.new_theta = [0.0] * num_particles # lists are faster than arrays here
        self.dist_jitter = 20 # mm was 50
        self.angle_jitter = 10 / 180 * pi # was 20 deg
        self.state = self.LOST
        self.update_pose_variance()

    def move(self):
        self.motion_model.move(self.particles)
        if self.sensor_model.evaluate(self.particles):  # true if log_weights changed
            weight_variance = self.update_weights()
            #print(f'pf move: weight_variance = {weight_variance}')
            self.resample()
            if self.state != ParticleFilter.LOCALIZED:
                print(';;; LOCALIZED AFTER MOVE ;;;')
                self.state = self.LOCALIZED
            self.update_pose_variance() # will also update the pose estimate
        else:
            self.update_pose_estimate()
        self.robot.world_map.update_held_object()

    def delocalize(self):
        self.state = self.LOST
        print('*** LOST ***')
        self.motion_model.last_pose = None
        self.randomizer.initialize(self.robot)
        self.update_pose_variance()

    def update_pose_estimate(self):
        cx = 0.0; cy = 0.0
        hsin = 0.0; hcos = 0.0
        weight_sum = 0.0
        best_particle = self.particles[0]
        for p in self.particles:
            p.weight = exp(p.log_weight)
            if p.weight > best_particle.weight:
                best_particle = p
            cx += p.x * p.weight
            cy += p.y * p.weight
            hsin += sin(p.theta) * p.weight
            hcos += cos(p.theta) * p.weight
            weight_sum += p.weight
        if weight_sum == 0:
            weight_sum = 1
        cx /= weight_sum
        cy /= weight_sum
        self.pose = PoseEstimate(cx, cy, 0, atan2(hsin,hcos))
        self.best_particle = best_particle
        self.robot.pose = self.pose
        return self.pose

    def update_pose_variance(self):
        weight = var_xx = var_xy = var_yy = r_sin = r_cos = 0.0
        pose = self.update_pose_estimate()
        mu_x = pose.x; mu_y = pose.y; mu_theta = pose.theta
        for p in self.particles:
            dx = (p.x - mu_x)
            dy = (p.y - mu_y)
            var_xx += dx * dx * p.weight
            var_xy += dx * dy * p.weight
            var_yy += dy * dy * p.weight
            r_sin += sin(p.theta) * p.weight
            r_cos += cos(p.theta) * p.weight
            weight += p.weight
        xy_var = np.array([[var_xx, var_xy],
                           [var_xy, var_yy]]) / weight
        Rsq = r_sin**2 + r_cos**2
        Rav = sqrt(Rsq) / weight
        theta_var = max(0, 1 - Rav)
        self.variance = (xy_var, theta_var)
        #print('update_pose_variance:', pose, self.variance)
        rough_var = max(abs(var_xx), abs(var_yy)) / weight
        #print('rough_var=', rough_var, 'state=', self.state)
        if rough_var > 1000:
            if self.robot.particle_filter.state != self.robot.particle_filter.LOST:
                print('*** LOST DUE TO HIGH POSE VARIANCE ***')
                self.state = ParticleFilter.LOST
        elif rough_var > 100:
            print('*** LOCALIZING IN PROGRESS: POSE VARIANCE > 100 ***')
            self.state = ParticleFilter.LOCALIZING
        return self.variance

    def update_weights(self):
        # Clip the log_weight values and calculate the new weights.
        particles = self.particles
        max_weight = max(p.log_weight for p in particles)
        if max_weight >= self.min_log_weight:
            wt_inc = 0.0
        else:
            wt_inc = - self.min_log_weight / 2.0
            print('wt_inc',wt_inc,'applied for max_weight',max_weight)
        exp_weights = self.exp_weights
        for i in range(self.num_particles):
            p = particles[i]
            p.log_weight += wt_inc
            exp_weights[i] = p.weight = exp(p.log_weight)
        weight_variance = np.var(exp_weights)
        return weight_variance

    def resample(self):
        # Compute and normalize the cdf; make local pointers for faster access.
        #print('resampling...')
        exp_weights = self.exp_weights
        cdf = self.cdf
        cumsum = 0
        for i in range(self.num_particles):
            cumsum += exp_weights[i]
            cdf[i] = cumsum
        np.divide(cdf,cumsum,cdf)

        # Resampling loop: choose particles to spawn
        uincr = 1.0 / self.num_particles
        u = random.random() * uincr
        index = 0
        new_indices = self.new_indices
        for j in range(self.num_particles):
            while u > cdf[index]:
                index += 1
            new_indices[j] = index
            u += uincr

        self.install_new_particles()

    def install_new_particles(self):
        particles = self.particles
        new_indices = self.new_indices
        new_x = self.new_x
        new_y = self.new_y
        new_theta = self.new_theta
        for i in range(self.num_particles):
            p = particles[new_indices[i]]
            new_x[i] = p.x
            new_y[i] = p.y
            new_theta[i] = p.theta
        for i in range(self.num_particles):
            p = particles[i]
            p.x = new_x[i]
            p.y = new_y[i]
            p.theta = new_theta[i]
            p.log_weight = 0.0
            p.weight = 1.0

    def set_pose(self,x,y,theta):
        for p in self.particles:
            p.x = x
            p.y = y
            p.theta = theta
            p.log_weight = 0.0
            p.weight = 1.0
        self.state = ParticleFilter.LOCALIZED
        self.update_pose_variance()

    def look_for_new_landmarks(self): pass  # SLAM only

    def clear_landmarks(self):
        print('clear_landmarks: Landmarks are fixed in this particle filter.')

    def increase_variance(self):
        TRANSLATION_NOISE = 10 # mm
        ROTATION_NOISE = 0.1 # radians
        for p in self.particles:
            p.x += (np.random.random() - 0.5) * TRANSLATION_NOISE
            p.y += (np.random.random() - 0.5) * TRANSLATION_NOISE
            p.theta += (np.random.random() - 0.5) * ROTATION_NOISE

    #================ "show" commands that can be used by simple_cli

    def show_landmarks(self):
        landmarks = self.sensor_model.landmarks
        print('The particle filter has %d landmark%s:' %
              (len(landmarks), '' if (len(landmarks) == 1) else 's'))
        self.show_landmarks_workhorse(landmarks)

    def show_landmarks_workhorse(self,landmarks):
        "Also called by show_particle"
        for key in sorted(landmarks.keys()):
            value = landmarks[key]
            if isinstance(value, Pose):
                x = value.x
                y = value.y
                theta = value.theta
                sigma_x = 0
                sigma_y = 0
                sigma_theta = 0
            else:
                x = value[0][0,0]
                y = value[0][1,0]
                theta = value[1] * 180/pi
                sigma_x = sqrt(value[2][0,0])
                sigma_y = sqrt(value[2][1,1])
                sigma_theta = sqrt(value[2][2,2])*180/pi
            if key.startswith('ArucoMarker-'):
                print('  Aruco marker %s' % key[12:], end='')
            else:
                print('  %r' % key, end='')
            print(' at (%6.1f, %6.1f) @ %4.1f deg    +/- (%4.1f,%4.1f)  +/- %3.1f deg' %
                  (x, y, theta, sigma_x, sigma_y, sigma_theta))
        print()

    def show_particle(self,args=[]):
        if len(args) == 0:
            particle = self.best_particle
            particle_number = '(best=%d)' % particle.index
        elif len(args) > 1:
            print('Usage:  show particle [number]')
            return
        else:
            try:
                particle_number = int(args[0])
                particle = self.particles[particle_number]
            except ValueError:
                print('Usage:  show particle [number]')
                return
            except IndexError:
                print('Particle number must be between 0 and',
                      len(self.particles)-1)
                return
        print ('Particle %s:  x=%6.1f  y=%6.1f  theta=%6.1f deg   log wt=%f [%.25f]' %
               (particle_number, particle.x, particle.y, particle.theta*180/pi,
                particle.log_weight, particle.weight))
        if isinstance(particle,SLAMParticle) and len(particle.landmarks) > 0:
            print('Landmarks:')
            self.show_landmarks_workhorse(particle.landmarks)
        else:
            print()


###should be enough till here for particle filter alone###

#================ Particle SLAM ================

class SLAMParticle(Particle):
    def __init__(self, index=-1):
        super().__init__(index)
        self.landmarks = dict()

    def __repr__(self):
        return '<SLAMParticle %d: (%.2f, %.2f) %.1f deg. log_wt=%f, %d-lm>' % \
               (self.index, self.x, self.y, self.theta*180/pi, self.log_weight, len(self.landmarks))

    sigma_r = 50
    sigma_alpha = 15 * (pi/180)
    sigma_phi = 15 * (pi/180)
    sigma_theta =  15 * (pi/180)
    sigma_z = 50
    # sigma_r = 10
    # sigma_alpha = 5 * (pi/180)
    # sigma_phi = 15 * (pi/180)
    # sigma_theta =  5 * (pi/180)
    # sigma_z = 10

    landmark_sensor_variance_Qt = np.array([[sigma_r**2, 0             , 0],
                                            [0         , sigma_alpha**2, 0],
                                            [0         , 0             , sigma_phi**2]])
    # variance of camera location (cylindrical coordinates)
    # phi is the angle around the Z axis of the robot
    # theta is the angle around the X axis of the camera (pitch)
    camera_sensor_variance_Qt = np.array([[sigma_r**2 , 0             , 0          ,0           , 0],
                                          [0          , sigma_alpha**2, 0          ,0           , 0],
                                          [0          , 0             , sigma_z**2 ,0           , 0],
                                          [0          , 0             , 0          ,sigma_phi**2, 0],
                                          [0          , 0             , 0          ,0           , sigma_theta**2]])

    @staticmethod
    def sensor_jacobian_H(dx, dy, dist):
        """Jacobian of sensor values (r, alpha) wrt particle state x,y
           where (dx,dy) is vector from particle to lm, and
           r = sqrt(dx**2 + dy**2), alpha = atan2(dy,dx), phi = phi"""
        q = dist**2
        sqr_q = dist
        return np.array([[dx/sqr_q, dy/sqr_q, 0],
                         [-dy/q   , dx/q    , 0],
                         [0       , 0       , 1]])

    @staticmethod
    def sensor_jacobian_H_cam(dx, dy, dist):
        """Jacobian of sensor values (r, alpha) wrt particle state x,y
           where (dx,dy) is vector from particle to lm, and
           r = sqrt(dx**2 + dy**2), alpha = atan2(dy,dx), z = z, phi = phi, theta = theta"""
        q = dist**2
        sqr_q = dist
        return np.array([[dx/sqr_q, dy/sqr_q, 0, 0, 0],
                         [-dy/q   , dx/q    , 0, 0, 0],
                         [0       , 0       , 1, 0, 0],
                         [0       , 0       , 0, 1, 0],
                         [0       , 0       , 0, 0, 1],])

    def add_regular_landmark(self, lm_id, sensor_dist, sensor_bearing, sensor_orient):
        direction = self.theta + sensor_bearing
        dx = sensor_dist * cos(direction)
        dy = sensor_dist * sin(direction)
        lm_x = self.x + dx
        lm_y = self.y + dy

        if lm_id.startswith('ArucoMarker-') or lm_id.startswith('Wall-'):
            lm_orient = wrap_angle(sensor_orient + self.theta)
        else:
            print('Unrecognized landmark type:',lm_id)
            lm_orient = sensor_orient
        if self.index < 0:
            print(f'theta={self.theta*180/pi} sensor_orient={sensor_orient*180/pi}  lm_orient={lm_orient*180/pi}')
        lm_mu =  np.array([[lm_x], [lm_y]])
        H = self.sensor_jacobian_H(dx, dy, sensor_dist)
        Hinv = np.linalg.inv(H)
        Q = self.landmark_sensor_variance_Qt
        lm_sigma = Hinv.dot(Q.dot(Hinv.T))
        self.landmarks[lm_id] = (lm_mu, lm_orient, lm_sigma)

    def update_regular_landmark(self, id, sensor_dist, sensor_bearing, sensor_orient,
                               dx, dy, I=np.eye(3)):
        # (dx,dy) is vector from particle to SENSOR position of lm
        (old_mu, old_orient, old_sigma) = self.landmarks[id]
        H = self.sensor_jacobian_H(dx, dy, sensor_dist)
        Ql =  H.dot(old_sigma.dot(H.T)) + self.landmark_sensor_variance_Qt
        Ql_inv = np.linalg.inv(Ql)
        K = old_sigma.dot((H.T).dot(Ql_inv))
        z = np.array([[sensor_dist], [sensor_bearing], [sensor_orient + self.theta]])
        # (ex,ey) is vector from particle to map position of lm
        ex = old_mu[0,0] - self.x
        ey = old_mu[1,0] - self.y
        h = np.array([ [sqrt(ex**2+ey**2)],
                       [wrap_angle(atan2(ey,ex) - self.theta)],
                       [old_orient] ])
        delta_sensor = wrap_selected_angles(z-h, [1,2])
        if False: """#abs(delta_sensor[1,0]) > 0.1 or abs(delta_sensor[0,0]) > 50:
            # Huge delta means the landmark must have moved, so reset our estimate.
            if isinstance(id,str): # *** DEBUG
                print('update_regular_landmark: index=%d id=%s  dist=%5.1f  brg=%5.1f  orient=%5.1f' %
                      (self.index, id, sensor_dist, sensor_bearing*180/pi, sensor_orient*180/pi), end='')
                print('  delta sensor: %.1f  %.1f  %.1f' %
                      (delta_sensor[0,0], delta_sensor[1,0]*180/pi, delta_sensor[2,0]*180/pi))
            new_mu = np.array([[self.x + sensor_dist*cos(sensor_bearing+self.theta)],
                               [self.y + sensor_dist*sin(sensor_bearing+self.theta)],
                               [sensor_orient]])
            Hinv = np.linalg.inv(H)
            Q = self.landmark_sensor_variance_Qt
            new_sigma = Hinv.dot(Q.dot(Hinv.T))"""
        else:
            # Error not too large: refine current estimate using EKF
            new_mu = np.append(old_mu,[old_orient]).reshape([3,1]) + K.dot(delta_sensor)
            new_sigma = (I - K.dot(H)).dot(old_sigma)
        # landmark tuple is ( [x,y], orient, covariance_matrix )
        if self.index == -1:  # NOOP: should be == 0
            print('id=',id,'  old_mu=',[old_mu[0,0],old_mu[1,0]],'@',old_orient*180/pi,
                  '  new_mu=',[new_mu[0][0],new_mu[1][0]],'@',new_mu[2][0]*180/pi)
            print('   ','dx,dy=',[dx,dy],'  ex,ey=',[ex,ey],
                  ' sensor_dist=',sensor_dist,
                  ' sensor_bearing=',sensor_bearing*180/pi,
                  ' sensor_orient=',sensor_orient*180/pi,
                  ' delta_sensor=',delta_sensor.tolist())
        #print (f'Updating {id} to ({new_mu[0][0]},{new_mu[1][0]}) @ {new_mu[2][0]*180/pi}')
        self.landmarks[id] = (new_mu[0:2], wrap_angle(new_mu[2][0]), new_sigma)

class SLAMSensorModel(SensorModel):
    @staticmethod
    def is_solo_aruco_landmark(x):
        return isinstance(x, ArucoMarkerObj)

    @staticmethod
    def is_wall_landmark(x):
        return isinstance(x, WallObj)

    def __init__(self, robot, landmark_test=None, landmarks=None,
                 distance_variance=200):
        if landmarks is None:
            landmarks = dict()
        if landmark_test is None:
            landmark_test = self.is_wall_landmark # self.is_solo_aruco_landmark
        self.landmark_test = landmark_test
        self.distance_variance = distance_variance
        super().__init__(robot,landmarks)

    def evaluate(self, particles, force=False, just_looking=False):
        # Returns true if particles were evaluated.  Call with
        # force=True (e.g., from particle_viewer) to skip distance
        # traveled check.  Call with just_looking=True to just look
        # for new landmarks without particle evaluation.
        evaluated = False

        # Don't evaluate if robot is still moving; ArucoMarker info will be bad.
        if self.robot.is_moving() or self.pf.robot.world_map.visibility_paused:
            return False

        # Compute robot motion even if forced, to check for robot origin_id change
        (distance, turn_angle) = self.motion_since_last_evaluate()

        # If we're lost but have landmarks in view, see if we can
        # recover by using the landmarks to generate a new particle set.
        if self.pf.state == ParticleFilter.LOST:
            if self.pf.sensor_model.landmarks:
                found_lms = self.pf.make_particles_from_landmarks()
                if not found_lms:
                    return False
                else:
                    print('*** LOST BUT LOCALIZING')
                    self.pf.state = ParticleFilter.LOCALIZING
                    force = True
                    just_looking = False
            else: # no landmarks, so we can't be lost
                print(';;; LOCALIZED ;;; DUE TO ZERO LANDMARKS')
                self.pf.set_pose(0,0,0)
                self.pf.robot.world_map.clear()

                # Clear cliff sessions on delocalize (matches barrel/ball clearing behavior)
                if hasattr(self.pf.robot, "cliff_sessions"):
                    self.pf.robot.cliff_sessions = []
                if hasattr(self.pf.robot, "cliff_results"):
                    self.pf.robot.cliff_results = None
                    self.pf.robot.cliff_timestamp = None

        # Unless forced, don't evaluate unless the robot moved enough
        # for evaluation to be worthwhile.
        if (not force) and (distance < 5) and abs(turn_angle) < 2*pi/180:
            just_looking = True
        if not just_looking:
            self.last_evaluate_pose = self.robot.pose

        # Process seen landmarks or add new ones
        seen_landmarks = [obj for obj in self.robot.world_map.objects.values()
                               if self.landmark_test(obj) and obj.is_visible]
        for obj in seen_landmarks:
            evaluated = self.process_landmark(obj, just_looking) or evaluated

        if evaluated:
            wmax = - np.inf
            for p in particles:
                wmax = max(wmax, p.log_weight)
            if wmax > -5.0 and self.pf.state != ParticleFilter.LOCALIZED:
                print('::: LOCALIZED DUE TO EVALUATE :::')
                self.pf.state = ParticleFilter.LOCALIZED
            elif self.pf.state != ParticleFilter.LOCALIZED:
                print('not localized because wmax =', wmax)
            min_log_weight = self.robot.particle_filter.min_log_weight
            if wmax < min_log_weight:
                wt_inc = min_log_weight - wmax
                # print('wmax=',wmax,'wt_inc=',wt_inc)
                for p in particles:
                    p.log_weight += wt_inc
        return evaluated

    def process_landmark(self, obj, just_looking):
        particles = self.robot.particle_filter.particles
        rpose = self.robot.pose
        if not self.landmark_test(obj):
            return False
        if isinstance(obj, (ArucoMarkerObj, WallObj)):
            sensor_dist = obj.sensor_distance
            sensor_bearing = obj.sensor_bearing
            sensor_orient = obj.sensor_orient
        else:
            print("Don't know how to process landmark; id =",id)

        id = obj.id
        if id not in self.landmarks:
            if self.pf.state == ParticleFilter.LOCALIZED:
                print('  *** PF ADDING LANDMARK %s at:  distance=%6.1f  bearing=%5.1f deg.  orient=%5.1f deg.' %
                      (id, sensor_dist, sensor_bearing*180/pi, sensor_orient*180/pi))
                for p in particles:
                    p.add_regular_landmark(id, sensor_dist, sensor_bearing, sensor_orient)
                # Add new landmark to sensor model's landmark list so worldmap can reference it
                self.landmarks[id] = self.pf.best_particle.landmarks[id]
            return False

        # If we reach here, we're seeing a familiar landmark, so evaluate
        if just_looking:
            return False
        else:
            # We've moved a bit, so we should update every particle.
            pp = particles
            evaluated = True

        #print(f'process_landmark {obj} vis:{obj.is_visible} theta:{self.robot.pose.theta*180/pi:.2f} moving:{self.robot.is_moving()}')
        should_update_landmark = (not obj.is_fixed) and \
            (self.pf.state == ParticleFilter.LOCALIZED)

        for p in pp:
            # Use sensed bearing and distance to get particle's
            # prediction of landmark position in the world.  Compare
            # to its stored map position.
            sensor_direction = p.theta + sensor_bearing
            dx = sensor_dist * cos(sensor_direction)
            dy = sensor_dist * sin(sensor_direction)
            predicted_lm_x = p.x + dx
            predicted_lm_y = p.y + dy
            (lm_mu, lm_orient, lm_sigma) = p.landmarks[id]
            map_lm_x = lm_mu[0,0]
            map_lm_y = lm_mu[1,0]
            error_x = map_lm_x - predicted_lm_x
            error_y = map_lm_y - predicted_lm_y
            error1_sq = error_x**2 + error_y**2
            error2_sq = 0 # *** (sensor_dist * wrap_angle(sensor_orient - lm_orient))**2
            p.log_weight -= (error1_sq + error2_sq) / self.distance_variance
            # Update landmark in this particle's map
            if should_update_landmark:
                    p.update_regular_landmark(id, sensor_dist, sensor_bearing,
                                             sensor_orient, dx, dy)
        return evaluated

class SLAMParticleFilter(ParticleFilter):
    def __init__(self, robot, landmark_test=SLAMSensorModel.is_wall_landmark, **kwargs):
        if 'sensor_model' not in kwargs or kwargs['sensor_model'] == 'default':
            kwargs['sensor_model'] = SLAMSensorModel(robot, landmark_test=landmark_test)
        if 'particle_factory' not in kwargs:
            kwargs['particle_factory'] = SLAMParticle
        if 'initializer' not in kwargs:
            kwargs['initializer'] = RobotPosition(0,0,0)
        if 'randomizer' not in kwargs:
            kwargs['randomizer'] = RandomWithinRadius()
        super().__init__(robot, **kwargs)
        self.state = ParticleFilter.LOCALIZED
        self.initializer.pf = self
        self.randomizer.pf = self
        self.new_landmarks = [None] * self.num_particles

    def clear_landmarks(self):
        self.sensor_model.landmarks.clear()
        for p in self.particles:
            p.landmarks.clear()

    def add_fixed_landmark(self,landmark):
        mu = np.array([[landmark.x], [landmark.y]])
        theta = landmark.theta
        sigma = np.zeros([3,3])
        mu_theta_sigma = (mu, theta, sigma)
        for p in self.particles:
            p.landmarks[landmark.id] = mu_theta_sigma
        self.sensor_model.landmarks[landmark.id] = mu_theta_sigma

    def update_weights(self):
        var = super().update_weights()
        best_particle = self.particles[self.exp_weights.argmax()]
        #print('  weight update: BEST ==> ',best_particle)
        self.sensor_model.landmarks = best_particle.landmarks
        return var

    def install_new_particles(self):
        particles = self.particles  # make local for faster access
        new_landmarks = self.new_landmarks
        new_indices = self.new_indices
        for i in range(self.num_particles):
            new_landmarks[i] = particles[new_indices[i]].landmarks.copy()
        super().install_new_particles()
        for i in range(self.num_particles):
            particles[i].landmarks = new_landmarks[i]

    def get_seen_landmarks(self):
        """We need to use candidate objects from the worldmap because they
        have the latest sensor distance and orientation information.
        If the robot is delocalized, worldmap objects are not being
        updated so their sensor values are now stale."""
        result = dict()
        lm_ids = list(self.sensor_model.landmarks.keys())
        for obj in self.robot.world_map.candidates:
            for lm_id in lm_ids:
                if lm_id.startswith(obj.name):
                    result[lm_id] = obj
                    break
        return result

    def make_particles_from_landmarks(self):
        seen_landmarks = self.get_seen_landmarks()
        ids = list(seen_landmarks.keys())
        num_seen = len(seen_landmarks)
        if num_seen == 0:
            return False
        particles = self.particles
        phi_jitter = np.random.normal(0.0, self.angle_jitter, size=self.num_particles)
        x_jitter = np.random.uniform(-self.dist_jitter, self.dist_jitter, size=self.num_particles)
        y_jitter = np.random.uniform(-self.dist_jitter, self.dist_jitter, size=self.num_particles)
        theta_jitter = np.random.uniform(-self.angle_jitter/2, self.angle_jitter/2, size=self.num_particles)
        for i in range(self.num_particles):
            id = ids[i % num_seen]
            obj = seen_landmarks[id]
            sensor_distance = obj.sensor_distance
            sensor_bearing = obj.sensor_bearing
            sensor_orient = obj.sensor_orient
            lm_pose = self.sensor_model.landmarks[id]
            lm_x,lm_y = lm_pose[0][0,0], lm_pose[0][1,0]
            lm_theta = lm_pose[1]
            robot_theta = wrap_angle(lm_theta - sensor_orient)
            # phi is our absolute bearing to the landmark, independent of our orientation
            phi = wrap_angle(robot_theta + sensor_bearing + phi_jitter[i])
            p = particles[i]
            p.x = lm_x - sensor_distance * cos(phi) + x_jitter[i]
            p.y = lm_y - sensor_distance * sin(phi) + y_jitter[i]
            p.theta = wrap_angle(robot_theta + theta_jitter[i])
            if i < 0: # change to i<0 to disable
                print(f'  lm = {obj}' +
                      f'  lm_theta={lm_theta*180/pi:.2f}' +
                      f'  sensor_orient={sensor_orient*180/pi:.2f}' +
                      f'  rpose = {self.robot.pose}' +
                      f'  robot_theta={robot_theta*180/pi:.2f}' +
                      f'  sensor_distance={sensor_distance:.2f}' +
                      f'  sensor_bearing={sensor_bearing*180/pi:.2f}' +
                      f'  abs bearing={phi*180/pi:.2f}' +
                      f' pose = ({p.x:.2f}, {p.y:.2f}) @ {p.theta*180/pi:.2f}')
            if i < 0:  # change to i<0 to disable
                print('NEW PARTICLE %d: ' % i, p.x, p.y, p.theta*180/pi)
        self.state = ParticleFilter.LOCALIZING
        return True


    def look_for_new_landmarks(self):
        """Calls evaluate() to find landmarks and add them to the maps.
        Also updates existing landmarks."""
        self.sensor_model.evaluate(self.particles, force=False, just_looking=True)
        self.sensor_model.landmarks = self.best_particle.landmarks
