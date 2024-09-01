from .geometry import *

class WorldObject():
    def __init__(self, id=None, x=0, y=0, z=0, is_visible=None):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.is_fixed = False   # True for walls and markers in predefined maps
        self.is_obstacle = True
        if is_visible is not None:
            self.is_visible = is_visible
        self.is_foreign = False
        if is_visible:
            self.pose_confidence = +1
        else:
            self.pose_confidence = -1

    def __repr__(self):
        return f'<{self.__class__.__name__} at ({self.x:.1f}, {self.y:.1f})>'


class OrangeBarrelObj(WorldObject):
    def __init__(self, spec):
        super().__init__()
        self.spec = spec
        self.name = spec['name']
        self.diameter = 22 # mm

class BlueBarrelObj(WorldObject):
    def __init__(self, spec):
        super().__init__()
        self.spec = spec
        self.name = spec['name']
        self.diameter = 22 # mm

class BallObj(WorldObject):
    def __init__(self, spec):
        super().__init__()
        self.spec = spec
        self.name = spec['name']

class RobotObj(WorldObject):
    def __init__(self, spec):
        super().__init__()
        self.spec = spec
        self.name = spec['name']


class ArucoMarkerObj(WorldObject):
    def __init__(self, aruco_parent, marker_number, id=None, x=0, y=0, z=0, theta=0):
        if id is None:
            id = 'Aruco-' + str(marker_number)
        super().__init__(id,x,y,z)
        self.aruco_parent = aruco_parent
        self.marker_number = marker_number
        self.theta = theta
        self.pose_confidence = +1

    @property
    def is_visible(self):
        return self.marker_number in self.aruco_parent.seen_marker_ids

    def __repr__(self):
        if self.pose_confidence >= 0:
            vis = ' visible' if self.is_visible else ''
            fix = ' fixed' if self.is_fixed else ''
            return '<ArucoMarkerObj %d: (%.1f, %.1f, %.1f) @ %d deg.%s%s>' % \
                (self.marker_number, self.x, self.y, self.z, self.theta*180/pi, fix, vis)
        else:
            return '<ArucoMarkerObj %d: position unknown>' % self.marker_number

################################################################

class WorldMap():

    def __init__(self,robot):
        self.robot = robot
        self.objects = dict()
        self.shared_objects = dict()
        self.aruco = None

    def clear(self):
        self.objects.clear()
        #self.robot.world.particle_filter.clear_landmarks()

    def update(self):
        objspecs = self.robot.robot0._ws_status_thread.current_status['aivision']['objects']['items']
        for spec in objspecs:
            if spec['name'] not in self.objects:
                obj = self.make_object(spec)
                self.objects[obj.name] = obj
                print(f"Created {obj}")
            self.update_object(spec)

    def make_object(self, spec):
        if spec['name'] == 'OrangeBarrel':
            obj = OrangeBarrelObj(spec)
        elif spec['name'] == 'BlueBarrel':
            obj = BlueBarrelObj(spec)
        elif spec['name'] == 'Ball':
            obj = BallObj(spec)
        elif spec['name'] == 'Robot':
            obj = RobotObj(spec)
        else:
            print(f"ERROR **** spec = {spec}")
            obj = None
        return obj

    def update_object(self, spec):
        cx = (spec['originx'] + spec['width']/2) * 2
        cy = (spec['originy'] + spec['height']) * 2
        hit = self.robot.kine.project_to_ground(cx, cy)
        obj = self.objects[spec['name']]
        robotpos = point(self.robot.x, self.robot.y)
        objpos = aboutZ(self.robot.theta*pi/180).dot(hit) + robotpos
        obj.x = objpos[0][0]
        obj.y = objpos[1][0]

