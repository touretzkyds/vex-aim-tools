import math
import copy
import numpy as np
import time
import datetime
import threading
import cv2

from .geometry import *
from .utils import *
from .camera import AIVISION_RESOLUTION_SCALE

class WorldObject():
    def __init__(self, id=None, name=None, x=0, y=0, z=0, theta=None, is_visible=False, is_fixed=False):
        self.id = id
        self.pose = PoseEstimate(x, y, z, theta)
        self.name = name or self.__class__.__name__
        self.matched = None  # matching object from data association
        self.is_fixed = is_fixed   # True for walls and markers in predefined maps
        self.is_obstacle = True # for path planning
        self.is_visible = is_visible
        self.is_missing = False # expect to see it but we don't
        self.is_valid = True
        self.held_by = None
        self.is_foreign = False # for shared maps; unused for now
        if is_visible:
            self.pose_confidence = +1
        else:
            self.pose_confidence = -1

    def __repr__(self):
        vis = 'visible' if self.is_visible else 'missing' if self.is_missing else 'unseen'
        held = " held" if self.held_by else ""
        return f'<{self.id or self.name} {vis} at ({self.pose.x:.1f}, {self.pose.y:.1f}){held}>'

    def update_matched_object(self,robot):
        "Update the matched world_map object with info from this candidate."
        self.matched.is_visible = True
        if self.matched.is_fixed or robot.particle_filter.state != robot.particle_filter.LOCALIZED:
            return
        MIN_MEASUREMENT_NOISE = 5
        measurement_noise = max(MIN_MEASUREMENT_NOISE, math.sqrt(self.sensor_distance)/2)
        if self.matched is not robot.holding:  # pose update will be done by update_held_object()
            self.matched.pose.update(self.pose, measurement_noise)
        if hasattr(self, 'spec'):
            self.matched.spec = self.spec
        if hasattr(self, 'marker'):
            self.matched.marker = self.marker
        if hasattr(self, 'seen_markers'):
            self.matched.seen_markers = self.seen_markers
        if hasattr(self, 'sensor_distance'):
            self.matched.sensor_distance = self.sensor_distance
        if hasattr(self, 'sensor_bearing'):
            self.matched.sensor_bearing = self.sensor_bearing
        if hasattr(self, 'sensor_orient'):
            self.matched.sensor_orient = self.sensor_orient
        if hasattr(self, 'wall'):
            self.matched.wall = self.wall.matched

class BarrelObj(WorldObject):
    def __init__(self, spec=None, id=None, x=0, y=0):
        if id is None and spec and 'id' in spec and isinstance(spec['id'], str):
            id = spec['id']
        super().__init__(id=id, x=x, y=y)
        self.spec = spec
        self.name = spec['name']
        self.diameter = 22 # mm
        self.height = 25 # mm

class OrangeBarrelObj(BarrelObj):
    pass

class BlueBarrelObj(BarrelObj):
    pass

class SportsBallObj(WorldObject):
    def __init__(self, spec=None, id=None, x=0, y=0):
        if id is None and spec and 'id' in spec and isinstance(spec['id'], str):
            id = spec['id']
        super().__init__(id=id, x=x, y=y)
        self.spec = spec
        self.name = spec['name']
        self.diameter = 25.0 # mm
        self.z = self.diameter / 2

class RobotObj(WorldObject):
    def __init__(self, spec=None, id=None, x=0, y=0, theta=0):
        super().__init__(id=id, x=x, y=y, theta=theta)
        self.spec = spec
        self.name = spec['name']

class AprilTagObj(WorldObject):
    def __init__(self, spec=None, id=None, x=0, y=0, theta=0):
        if id is None and spec and 'id' in spec:
            id = spec['id']
        super().__init__(id=id, x=x, y=y, theta=theta)
        self.spec = spec
        self.name = spec['name']
        self.tag_id = spec['id']
        self.base_diameter = 22 # mm
        self.width = 38 # mm

    def __repr__(self):
        vis = 'visible' if self.is_visible else 'missing' if self.is_missing else 'unseen'
        return f'<{self.id or self.name} {vis} at ({self.pose.x:.1f}, {self.pose.y:.1f}) @ {self.pose.theta*180/pi:.1f} deg.>'
    

class AprilTag0Obj(AprilTagObj):
    pass

class AprilTag1Obj(AprilTagObj):
    pass

class AprilTag2Obj(AprilTagObj):
    pass

class AprilTag3Obj(AprilTagObj):
    pass

class AprilTag4Obj(AprilTagObj):
    pass

class ArucoMarkerObj(WorldObject):
    def __init__(self, spec, x=0, y=0, z=0, theta=0, **kwargs):
        super().__init__(x=x, y=y, z=z, theta=theta, **kwargs)
        self.name = spec['name']
        self.marker = spec['marker']
        self.marker_id = spec['id']
        self.marker_string = 'ArucoMarker-' + str(spec['id'])
        self.pose_confidence = +1

    def __repr__(self):
        if self.pose_confidence >= 0:
            vis = 'visible' if self.is_visible else 'missing' if self.is_missing else 'unseen'
            fix = ' fixed' if self.is_fixed else ''
            return '<ArucoMarkerObj %s: (%.1f, %.1f, %.1f) @ %d deg.%s %s>' % \
                (self.marker_id, self.pose.x, self.pose.y, self.pose.z, self.pose.theta*180/pi, fix, vis)
        else:
            return f'<ArucoMarkerObj {self.marker_id}: position unknown>'
        

class WallObj(WorldObject):

    def __init__(self, wall_spec, x=0, y=0, z=0, theta=0):
        super().__init__(x=x, y=y, z=z, theta=theta)
        self.wall_spec = wall_spec
        self.name = wall_spec.label
        self.length = wall_spec.length
        self.height = wall_spec.height
        self.is_fixed = False

    def __repr__(self):
        vis = 'visible' if self.is_visible else 'unseen'
        return f'<WallObj {self.name} ({self.pose.x:.1f}, {self.pose.y:.1f}) @ {self.pose.theta*180/pi:.1f} deg. {vis}>'

    ALIGNMENT_THRESHOLD = 25 * pi/180 # 25 degrees: aruco markers can only differ by this much

    def is_wall_aligned(self, obj):
        """An aruco marker or candidate wall is wall-aligned if the
        sensor_orient values match, but could be off by pi if marker
        is on the back of the wall."""
        result = abs(wrap_angle(self.sensor_orient - obj.sensor_orient)) < self.ALIGNMENT_THRESHOLD or \
            (isinstance(obj,ArucoMarkerObj) and \
             abs(wrap_angle(self.sensor_orient + pi - obj.sensor_orient)) < self.ALIGNMENT_THRESHOLD)
        if result is False: pass
            # print(f'is_wall_aligned {self.name} {neaten(self.sensor_orient*180/pi)} with {obj.name}' +
            #       f' {neaten(obj.sensor_orient*180/pi)}' +
            #       f' diff = {neaten(abs(wrap_angle(self.sensor_orient - obj.sensor_orient))*180/pi)}  result: {result}')
        return result


class WallSpec():
    def __init__(self, wall_marker_dict, label=None, length=100, height=210, marker_specs=dict(), doorways=dict()):
        self.length = length
        self.height = height
        self.marker_specs = marker_specs
        self.doorways = doorways
        marker_id_numbers = list(marker_specs.keys())
        self.label = label or f'Wall-{min(marker_id_numbers)}'
        for id in marker_id_numbers:
            wall_marker_dict[id] = self
        wall_marker_dict[self.label] = self


class DoorwayObj(WorldObject):
    def __init__(self, wall, index):
        name = f'Doorway-{wall.name[5:]}:{index}'
        super().__init__(name=name, is_visible=wall.is_visible)
        door_spec = wall.wall_spec.doorways[index]
        self.door_width = door_spec['width']
        self.wall = wall
        self.index = index  # which doorway is this?  0, 1, ...
        self.is_obstacle = False
        self.update()

    def update(self):
        door_spec = self.wall.wall_spec.doorways[self.index]
        self.pose = copy.deepcopy(self.wall.pose)
        self.sensor_distance = self.wall.sensor_distance

    def __repr__(self):
        vis = 'visible' if self.is_visible else 'unseen'
        if self.pose_confidence >= 0:
            return '<DoorwayObj %s: (%.1f,%.1f) @ %.1f deg. %s>' % \
                (self.id, self.pose.x, self.pose.y, self.pose.theta*180/pi, vis)
        else:
            return '<DoorwayObj %s: position unknown>' % self.id

class RoomObj(WorldObject):
    def __init__(self, name,
                 points=np.resize(np.array([0,0,0,1]),(4,4)).transpose(),
                 floor=1, door_ids=[], connections=[]):
        "points should be four points in homogeneous coordinates forming a convex polygon"
        id = 'Room-' + name
        self.name = name
        x,y,z,s = points.mean(1)
        super().__init__(id=id, x=x, y=y)
        self.points = points
        self.floor = floor
        self.door_ids = door_ids
        self.connections = connections
        self.is_obstacle = False
        self.is_fixed = True

    def __repr__(self):
        return '<RoomObj %s: (%.1f,%.1f) floor=%s>' % (self.id, self.pose.x, self.pose.y, self.floor)

    def get_bounding_box(self):
        mins = self.points.min(1)
        maxs = self.points.max(1)
        return ((mins[0],mins[1]), (maxs[0],maxs[1]))


################################################################

class WorldMap():

    def __init__(self,robot):
        self.robot = robot
        self._lock = threading.RLock()
        self.objects = dict()
        self.pending_objects = dict()
        self.missing_objects = []
        self.shared_objects = dict()
        self.name_counts = dict()  # For generating new object names
        self.last_held_time = -1
        self.visibility_paused = False

    def __repr__(self):
        with self._lock:
            count = len(self.objects)
        return f'<WorldMap with {count} objects>'

    def snapshot_objects(self):
        with self._lock:
            snapshot = {}
            for key, obj in self.objects.items():
                try:
                    cloned = copy.copy(obj)
                    pose = getattr(obj, "pose", None)
                    if pose is not None:
                        try:
                            cloned.pose = PoseEstimate(pose)
                        except Exception:
                            cloned.pose = copy.copy(pose)
                    snapshot[key] = cloned
                except Exception:
                    snapshot[key] = obj
            return snapshot

    def clear(self):
        with self._lock:
            self.robot.particle_filter.clear_landmarks()
            self.objects.clear()
            self.pending_objects.clear()
            self.missing_objects = []
            self.shared_objects.clear()
            self.name_counts.clear()
        
    def pause_visibility(self, value=True):
        """Turn off visibility of objects when the robot is moving.  We won't
        turn it back on until the robot has stopped AND we have processed a new
        camera frame so visibilities are updated."""
        with self._lock:
            if self.visibility_paused != value:
                self.visibility_paused = value

    def update(self):
        with self._lock:
            if self.visibility_paused:
                return
            self.updated_objects = []
            self.make_new_objects_from_vision()
            self.associate_objects()
            self.update_associated_objects()
            self.detect_missing_objects()
            self.process_unassociated_objects()
            self.update_visibilities()
            self.update_holding()

    def make_new_objects_from_vision(self):
        self.candidates = list()
        self.make_new_aiv_objects()
        if self.robot.aruco_detector:
            self.make_new_wall_objects()
            self.make_new_aruco_objects()

    def make_vision_object(self, spec):
        if spec['name'] == 'OrangeBarrel':
            obj = OrangeBarrelObj(spec)
        elif spec['name'] == 'BlueBarrel':
            obj = BlueBarrelObj(spec)
        elif spec['name'] == 'SportsBall':
            obj = SportsBallObj(spec)
        elif spec['name'] == 'Robot':
            obj = RobotObj(spec)
        elif spec['name'].startswith('AprilTag'):
            obj = AprilTagObj(spec)
        else:
            print(f"ERROR **** spec = {spec}")
            return None
        return obj

    def make_new_aiv_objects(self):
        objspecs = self.robot.robot0.status['aivision']['objects']['items']
        for spec in objspecs:
            if spec['type_str'] == 'aiobj':
                base_name = spec['name']
            elif spec['type_str'] == 'tag':
                if 0 <= spec['id'] <= 4:
                    base_name = 'AprilTag-' + repr(spec['id'])
                    spec['name'] = base_name
                else:
                    #print('*** BAD TAG:', spec)
                    continue
            else:
                print(f'*** Unknown: spec={spec}')
                continue
            if spec['name'] == 'Robot':   # avoid spurious robot creation for now
                continue

            obj = self.make_vision_object(spec)
            obj.is_visible = True

            # Calculate midpoint of bottom edge, which we assume is on the floor
            height = spec['height']
            width = spec['width']
            cx = (spec['originx'] + width/2) * AIVISION_RESOLUTION_SCALE
            if isinstance(obj, BarrelObj):
                # correct height for possible occlusion by foreground object
                corr_height = max(height, width*1.10)
            elif isinstance(obj, AprilTagObj):
                corr_height = min(height, width)
            else:
                corr_height = height
            cy = (spec['originy'] + corr_height) * AIVISION_RESOLUTION_SCALE
            if isinstance(obj, AprilTagObj):
                TAG_TO_GROUND_CORRECTION = 1.25 # should be 2.0 but empirically 1.25 works better
                cy += corr_height * TAG_TO_GROUND_CORRECTION * AIVISION_RESOLUTION_SCALE
            hit = self.robot.kine.project_to_ground(cx, cy)
            angle = atan2(hit[1,0], hit[0,0])
            # offset hit by half the object thickness
            if obj.__dict__.get('diameter'):
                half_diameter = obj.diameter / 2
                increment = point(cos(angle) * half_diameter, sin(angle) * half_diameter, 0)
                hit += increment
            # convert to world coordinates
            robotpos = point(self.robot.pose.x, self.robot.pose.y)
            objpos = aboutZ(self.robot.pose.theta).dot(hit) + robotpos
            x = objpos[0][0]
            y = objpos[1][0]
            distance = ((x - self.robot.pose.x)**2 + (y - self.robot.pose.y)**2) ** 0.5
            MAX_DISTANCE = 300 # anything further than this is a spurious detection
            if distance > MAX_DISTANCE:
                continue
            obj.sensor_distance = distance
            if isinstance(obj, AprilTagObj):
                tag_angle_correction_factor = 4  # guesstimate
                angle = spec['angle'] - (0 if spec['angle'] < 180 else 360)
                theta = wrap_angle(self.robot.pose.theta + pi - angle / 180 * pi * tag_angle_correction_factor)
            else:
                theta = None
            obj.pose = Pose(x, y, 0, theta)
            if self.check_spec_indicates_held(obj):
                self.reposition_held_object(obj)
            self.candidates.append(obj)

    def make_new_aruco_objects(self):
        camera_offset_vector = np.array([0, 0, self.robot.kine.camera_from_origin])
        detector = self.robot.aruco_detector
        if hasattr(detector, "snapshot_seen_markers"):
            seen_markers = detector.snapshot_seen_markers()
        else:
            seen_markers = detector.seen_marker_objects.copy()
        for (id,marker) in seen_markers.items():
            name = f'ArucoMarker-{id}'
            spec = {'name': name, 'id': id, 'marker': marker}
            sensor_coords = marker.camera_coords + camera_offset_vector
            sensor_distance = math.sqrt(sensor_coords[0]**2 + sensor_coords[2]**2)
            sensor_bearing = atan2(sensor_coords[0], sensor_coords[2])
            sensor_orient = wrap_angle(pi - marker.euler_angles[1])
            theta = self.robot.pose.theta
            obj = ArucoMarkerObj(spec)
            obj.pose = Pose(self.robot.pose.x + sensor_distance * cos(theta + sensor_bearing),
                            self.robot.pose.y + sensor_distance * sin(theta + sensor_bearing),
                            marker.aruco_parent.marker_size / 2,  # *** TEMPORARY HACK ***
                            wrap_angle(self.robot.pose.theta + sensor_orient))
            obj.sensor_distance = sensor_distance
            obj.sensor_bearing = sensor_bearing
            obj.sensor_orient = sensor_orient
            obj.is_visible = True
            self.candidates.append(obj)

    def make_new_wall_objects(self):
        detector = self.robot.aruco_detector
        if hasattr(detector, "snapshot_seen_markers"):
            seen = detector.snapshot_seen_markers()
        else:
            seen = detector.seen_marker_objects.copy()
        wall_markers = dict()
        for (id,marker) in seen.items():
            if id in self.robot.world_map.wall_marker_dict:
                spec = self.robot.world_map.wall_marker_dict[id]
                if spec.label not in wall_markers:
                    wall_markers[spec.label] = list()
                wall_markers[spec.label].append((id,marker))
        for (wall_id, markers) in wall_markers.items():
            orients = [marker[1].euler_angles[1] for marker in markers]
            orig_orients = copy.copy(orients)
            if len(orients) == 1:
                # one marker is enough to update a wall if we're localized
                if self.robot.particle_filter.state != self.robot.particle_filter.LOCALIZED:
                    continue
            elif len(orients) == 2:
                if abs(wrap_angle(orients[0] - orients[1])) > WallObj.ALIGNMENT_THRESHOLD:
                    # with two markers that disagree, we can't tell which is the outlier, so punt
                    #print(f'wall {wall_id} marker outlier: {[o*180/pi for o in orients]}')
                    continue
            else:
                orients_consistent = False
                while not orients_consistent:
                    n = len(orients)
                    orients_consistent = True
                    for i in range(n):
                        exceeds = [abs(wrap_angle(orients[i] - orients[(i+j+1)%n])) > WallObj.ALIGNMENT_THRESHOLD
                                   for j in range(n-1)]
                        if all(exceeds):
                            #print('marker',i,' outlier:', orients)
                            del orients[i]
                            del markers[i]
                            orients_consistent = False
                            break
                if len(orients) < 2:
                    print('outlier removal left us one marker:', markers)
            wall = self.infer_wall_from_corners_lists(wall_id, markers)
            if wall is None:
                continue
            wall.aruco_orients = orients
            wall.seen_markers = markers
            self.candidates.append(wall)
            #print('candidate:', wall, 'orients(deg)=', [o*180/pi for o in orients])
            # Don't make doorways until wall is confirmed in world map
            if [k for k in self.objects.keys() if k.startswith(wall.name)]:
                self.make_doorways_from_wall(wall)

    def infer_wall_from_corners_lists(self, wall_id, markers):
        wall_spec = self.robot.world_map.wall_marker_dict[wall_id]
        marker_size = self.robot.aruco_detector.marker_size
        world_points = []
        image_points = []
        last_solution = None
        for (id, marker) in markers:
            length = wall_spec.length
            side = wall_spec.marker_specs[id]['side']
            cx = wall_spec.marker_specs[id]['x']
            cy = wall_spec.marker_specs[id]['y']
            world_points.append((cx-marker_size/2 - length/2, cy+marker_size/2, 0.))
            world_points.append((cx+marker_size/2 - length/2, cy+marker_size/2, 0.))
            world_points.append((cx+marker_size/2 - length/2, cy-marker_size/2, 0.))
            world_points.append((cx-marker_size/2 - length/2, cy-marker_size/2, 0.))

            corners = marker.corners[0]
            image_points.append(corners[0])
            image_points.append(corners[1])
            image_points.append(corners[2])
            image_points.append(corners[3])

            # Find rotation and translation vector from camera frame using SolvePnP
            try:
                (success, rvec, tvec) = cv2.solvePnP(np.array(world_points, dtype=np.float64),
                                                     np.array(image_points, dtype=np.float64),
                                                     self.robot.camera.camera_matrix,
                                                     self.robot.camera.distortion_array)
            except Exception as e:
                print('*** SolvePnP exception', e, '\n',
                      'world_points=', world_points, '\n',
                      'image_points=', image_points)
                continue
            if success:
                last_solution = (rvec, tvec, side)
        if last_solution is None:
            return None
        rvec, tvec, side = last_solution
        rotationm, jacob = cv2.Rodrigues(rvec)
        euler_angles = rotation_matrix_to_euler_angles(rotationm)
        wall_orient = euler_angles[1]
        tvec[2][0] += self.robot.kine.camera_from_origin  # want distance from base frame not camera

        sensor_coords = (-tvec[0,0], -tvec[1,0], tvec[2,0])
        sensor_distance = math.sqrt(sensor_coords[0]**2 + sensor_coords[2]**2)
        sensor_bearing = atan2(sensor_coords[0], sensor_coords[2])
        # Flip wall orientation to match ArUcos for worldmap
        sensor_orient = wrap_angle(pi - wall_orient) if side > 0 else -wall_orient
        theta = self.robot.pose.theta
        wall = WallObj(wall_spec)
        wall.pose = Pose(self.robot.pose.x + sensor_distance * cos(theta + sensor_bearing),
                         self.robot.pose.y + sensor_distance * sin(theta + sensor_bearing),
                         0,
                         wrap_angle(self.robot.pose.theta + sensor_orient))
        wall.sensor_distance = sensor_distance
        wall.sensor_bearing = sensor_bearing
        wall.sensor_orient = sensor_orient
        wall.is_visible = True
        return wall

    def make_doorways_from_wall(self, wall):
        for (index, door_spec) in wall.wall_spec.doorways.items():
            door = DoorwayObj(wall, index)
            self.candidates.append(door)

    def generate_doorway_list(self):
        "Used by path-planner.py"
        doorways = []
        for (key,obj) in self.objects.items():
            if isinstance(obj,DoorwayObj):
                w = obj.door_width / 2
                doorway_threshold_theta = obj.pose.theta + pi/2
                dx = w * cos(doorway_threshold_theta)
                dy = w * sin(doorway_threshold_theta)
                ox = obj.pose.x
                oy = obj.pose.y
                doorways.append((obj, ((ox-dx, oy-dy), (ox+dx, oy+dy))))
        return doorways

    def associate_objects(self):
        obj_types = list(set(type(obj) for obj in self.candidates))
        for otype in obj_types:
            self.associate_objects_of_type(otype)

    def association_cost(self, new_obj, old_obj):
        if isinstance(new_obj, WallObj) and len(new_obj.aruco_orients) < 2 \
           and not new_obj.is_wall_aligned(old_obj):
            cost = np.inf
        else:
            cost = ((new_obj.pose.x - old_obj.pose.x)**2 + (new_obj.pose.y - old_obj.pose.y)**2)
        return cost

    def associate_objects_of_type(self, otype):
        new = [c for c in self.candidates if type(c) is otype]
        old = [o for o in self.objects.values() if type(o) is otype]
        N_new = len(new)
        N_old = len(old)
        if N_old == 0:
            return
        costs = np.zeros([N_new,N_old])
        if self.robot.particle_filter and \
           self.robot.particle_filter.state != self.robot.particle_filter.LOCALIZED:
            MAX_ACCEPTABLE_COST = np.inf
        elif otype in (ArucoMarkerObj, WallObj, DoorwayObj):
            MAX_ACCEPTABLE_COST = np.inf  # should adjust based on pf undertainty
        else:
            MAX_ACCEPTABLE_COST = 500  # should adjust based on pf undertainty
        for i in range(N_new):
            for j in range(N_old):
                if otype is ArucoMarkerObj and new[i].marker_id != old[j].marker_id:
                    costs[i,j] = MAX_ACCEPTABLE_COST + 1
                elif otype is AprilTagObj and new[i].tag_id != old[j].tag_id:
                    costs[i,j] = MAX_ACCEPTABLE_COST + 1
                else:
                    costs[i,j] = self.association_cost(new[i], old[j])
        # *** Greedy algorithm; replace with the Hungarian algorithm
        for i in range(N_new):
            bestj = costs[i,:].argmin()
            if costs[i,bestj] < MAX_ACCEPTABLE_COST:
                new[i].matched = old[bestj]
                costs[:,bestj] = 1 + MAX_ACCEPTABLE_COST
        #print(f'{old=}  {new=}  {costs=}  {new[0].matched=}')

    def update_associated_objects(self):
        for candidate in self.candidates:
            if candidate.matched:
                candidate.update_matched_object(self.robot)
                self.updated_objects.append(candidate.matched)
                candidate.matched.is_missing = False
                if candidate.matched in self.missing_objects:
                    self.missing_objects.remove(candidate.matched)

    def should_be_visible(self, obj):
        # Really crude approach for now.  Should be doing camera
        # projection and accounting for occlusion.  In the future we
        # should employ the depth map for occusion detection.
        # For now, just return true if the object's bearing is within
        # the camera field of view and the distance is not too large.
        dx = obj.pose.x - self.robot.pose.x
        dy = obj.pose.y - self.robot.pose.y
        bearing = wrap_angle(atan2(dy,dx) - self.robot.pose.theta)
        distance = (dx**2 + dy**2) ** 0.5
        DISTANCE_THRESHOLD = 400 # mm
        BEARING_THRESHOLD = 30 # degrees
        result = abs(bearing)*180/pi < BEARING_THRESHOLD and distance < DISTANCE_THRESHOLD
        return result

    def detect_missing_objects(self):
        for obj in self.objects.values():
            if not isinstance(obj, (ArucoMarkerObj,WallObj,DoorwayObj)) and \
               obj not in self.updated_objects and self.should_be_visible(obj):
                if obj not in self.missing_objects:
                    obj.is_visible = False
                    obj.is_missing = True
                    self.missing_objects.append(obj)
                    #print(f'missing object: {obj}, visibility_paused={self.visibility_paused} {self.updated_objects=}')

    def process_unassociated_objects(self):
        """
        The vision system produces lots of spurious objects, so we require
        a new object to be seen 6 times in successive camera frames before
        we add it to the world map.
        """
        unassociated = [c for c in self.candidates if c.matched is None]
        pending = list(self.pending_objects.keys())
        COST_THRESHOLD = 50
        if self.robot.particle_filter and \
           self.robot.particle_filter.state != self.robot.particle_filter.LOCALIZED:
            pass # return
        if self.robot.particle_filter:
            pass # print('robot.particle_filter.state=', self.robot.particle_filter.state)
        for candidate in unassociated:
            if isinstance(candidate, WallObj) and len(candidate.aruco_orients) == 1:
                #print('punting on', candidate, 'from', self.objects)
                # only one aruco isn't enough to make a new wall
                continue
            matches = [p for p in pending if self.association_cost(candidate,p) < COST_THRESHOLD]
            if matches:
                m = matches[0]
                self.pending_objects[m] += 1
                if self.pending_objects[m] >= 6:
                    if self.reclaim_object(candidate):
                        pass
                    else:
                        candidate.id = self.next_in_sequence(candidate.name)
                        candidate.pose = PoseEstimate(candidate.pose)
                        self.objects[candidate.id] = candidate
                        candidate.is_visible = True
                        print('Added', candidate)
                        self.updated_objects.append(candidate)
                    del self.pending_objects[m]
                pending.remove(m)
            else:
                self.pending_objects[candidate] = 1
        for p in pending:
            #print('retracted', p, '  count=', self.pending_objects[p])
            del self.pending_objects[p]

    def reclaim_object(self, obj):
        t = type(obj)
        missing = [m for m in self.missing_objects if type(m) == t]
        if hasattr(obj,'marker_id'):
            missing = [m for m in missing if m.marker_id == obj.marker_id]
        if hasattr(obj,'tag_id'):
            missing = [m for m in missing if m.tag_id == obj.tag_id]
        if len(missing) == 0:
            return None
        costs = [self.association_cost(obj, m) for m in missing]
        min_index = np.argmin(costs)
        match = missing[min_index]
        match.is_visible = True
        match.is_missing = False
        match.pose = PoseEstimate(obj.pose)
        self.updated_objects.append(match)
        self.missing_objects.remove(match)
        #print('reclaimed', match)
        return match
        
    def next_in_sequence(self,name):
        count = 1 + self.name_counts.get(name, 0)
        self.name_counts[name] = count
        return name + "." + self.to_base_26(count)

    def to_base_26(self, num):
        result = []
        while num > 0:
            num -= 1  # Adjust for 1-based indexing (A=1, Z=26)
            remainder = num % 26
            result.append(chr(remainder + ord('a')))
            num //= 26
            return ''.join(reversed(result))
    
    def update_visibilities(self):
        for obj in self.objects.values():
            if obj not in self.updated_objects:
                obj.is_visible = False

    def update_holding(self):
        if self.robot.holding:
            self.confirm_still_holding()
        else:
            self.confirm_not_holding()

    def confirm_still_holding(self):
        MIN_UNHOLDING_TIME = 0.75  # seconds
        t = time.time()
        if (isinstance(self.robot.holding, BarrelObj) and self.robot.robot0.has_any_barrel()) or \
            (isinstance(self.robot.holding, SportsBallObj) and self.robot.robot0.has_sports_ball()):
            self.last_held_time = t
        else:
            if t - self.last_held_time > MIN_UNHOLDING_TIME:
                # held object has been gone long enough
                print('No longer holding', self.robot.holding)
                self.robot.holding.held_by = None
                self.robot.holding = None
            else:
                pass # wait a bit to see if held object comes back

    def check_spec_indicates_held(self, obj):
        if not isinstance(obj, (BarrelObj,SportsBallObj)):
            return False
        if isinstance(obj, BarrelObj):
            max_left = 150
            min_right = 460
        else:
            max_left = 200
            min_right = 470
        spec = obj.spec
        #print(f"left={spec['originx']*AIVISION_RESOLUTION_SCALE}  {max_left=}  " + \
        #      f"right={(spec['originx'] + spec['width']) * AIVISION_RESOLUTION_SCALE} {min_right=}")
        #
        # never seen objects acquired from map layout won't have originx
        if 'originx' in spec and \
           spec['originx']*AIVISION_RESOLUTION_SCALE < max_left and \
           (spec['originx'] + spec['width']) * AIVISION_RESOLUTION_SCALE > min_right:
            return True
        else:
            return False

    def confirm_not_holding(self):
        if self.robot.robot0.has_any_barrel() or self.robot.robot0.has_sports_ball():
            held_obj = None
            for obj in self.objects.values():
                if obj.is_visible and self.check_spec_indicates_held(obj):
                    held_obj = obj
                    break
            if held_obj:
                print('Robot now holding', held_obj)
                self.robot.holding = held_obj
                held_obj.held_by = self.robot
            else:
                pass # print('*** Could not find held object.')

    def reposition_held_object(self, obj):
        r = self.robot.kine.body_diameter/2 + obj.diameter/2
        pt = aboutZ(self.robot.pose.theta).dot(point(r,0))
        obj.pose.x = self.robot.pose.x + pt[0,0]
        obj.pose.y = self.robot.pose.y + pt[1,0]

    def update_held_object(self):
        if self.robot.holding:
            self.reposition_held_object(self.robot.holding)

    def show_objects(self):
        with self._lock:
            objs = sorted(self.objects.items(), key=lambda x: x[0])
            if len(objs) == 0:
                print('No objects in the world map.\n')
                return
            width = max([len(x[0]) for x in objs])
            for obj in objs:
                print(f'{obj[0].rjust(width)}: {obj[1]}')
            print()



################ GPT interface ################

    def get_prompt(self):
        with self._lock:
            prompt = ''
            prompt += f'It is now {datetime.datetime.now().strftime("%B %d, %Y, %I:%M:%S %p")}.\n'
            if self.robot.particle_filter.state == self.robot.particle_filter.LOCALIZED:
                prompt += f'You are located at ({round(self.robot.pose.x)}, {round(self.robot.pose.y)})\n'
                prompt += f'Your heading is {round(self.robot.pose.theta*180/pi)} degrees\n'
            else:
                prompt += f'You are currently lost (not localized) and do not see any landmarks.\n'
            if self.robot.holding:
                prompt += f'You are currently holding {self.robot.holding.id}.\n'
            else:
                prompt += f'You are not currently holding anything.\n'
            prompt += f'Your battery level is {self.robot.battery_percentage} percent.\n'
            for (id,obj) in self.objects.items():
                if not obj.is_missing:
                    if obj.is_visible:
                        vis = "visible"
                    else:
                        vis = "not visible"
                    prompt += f'{id} is located at ({round(obj.pose.x)}, {round(obj.pose.y)}) ' + \
                        f'and is {vis}\n'
                else:
                    prompt += f'{id} is missing\n'

                if isinstance(obj, WallObj):
                    front_markers = []
                    back_markers = []
                    for marker_id, marker_info in obj.wall_spec.marker_specs.items():
                        if marker_info['side'] == 1:  # +1 means front side
                            front_markers.append(marker_id)
                        else:  # -1 means back side
                            back_markers.append(marker_id)
                    prompt += f'{obj.id} has markers {front_markers} on its front side and {back_markers} on its back side\n'   

                if isinstance(obj, DoorwayObj) and obj.wall:
                    prompt += f'{id} is part of {obj.wall.id}\n'
            landmark_ids = list(self.robot.particle_filter.sensor_model.landmarks.keys())
            if landmark_ids:
                for id in landmark_ids:
                    prompt += f'{id} is a navigation landmark.'
            else:
                prompt += 'There are currently no navigation landmarks.'    
            return prompt
