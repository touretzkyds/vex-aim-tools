import math
import numpy as np

from . import geometry
#from . import rrt_shapes

class Joint():
    def __init__(self, name, parent=None, type='fixed', getter=(lambda:0),
                 description='A kinematic joint',
                 qmin=-math.inf, qmax=math.inf,
                 d=0, theta=0, r=0, alpha=0,
                 collision_model=None, ctransform=geometry.identity()):
        self.name = name
        self.parent = parent
        self.type = type
        if type == 'fixed':
            self.apply_q = self.fixed
        elif type == 'revolute':
            self.apply_q = self.revolute
        elif type == 'prismatic':
            self.apply_q = self.prismatic
        elif type == 'world':
            self.apply_q = self.world_joint
        else:
            raise ValueError("Type must be 'fixed', 'revolute', 'prismatic', or 'world'.")
        self.getter = getter
        self.description = description
        self.children = []
        self.d = d
        self.theta = theta
        self.r = r
        self.alpha = alpha
        self.children = []
        self.collision_model = collision_model
        self.q = 0
        self.qmin = qmin
        self.qmax = qmax
        self.parent_link_to_this_joint = geometry.dh_matrix(-d,-theta,-r,-alpha)
        self.this_joint_to_parent_link = np.linalg.inv(self.parent_link_to_this_joint)

        self.solver = None

    def __repr__(self):
        if self.type == 'fixed':
            qval = 'fixed'
        elif isinstance(self.q, (int,float)):
            qval = "q=%.2f deg." % (self.q*180/math.pi)
        else:
            qval = ("q=%s" % repr(self.q))
        return "<Joint '%s' %s>" % (self.name, qval)

    def this_joint_to_this_link(self):
        "The link moves by q in the joint's reference frame."
        return self.apply_q()

    def this_link_to_this_joint(self):
        return np.linalg.inv(self.this_joint_to_this_link())

    def revolute(self):
        return geometry.aboutZ(-self.q)

    def prismatic(self):
        return geometry.translate(0.,0.,-self.q)

    def fixed(self):
        return geometry.identity()

    def world_joint(self):
        return geometry.translate(self.q[0],self.q[1]).dot(geometry.aboutZ(self.q[2]))

class Kinematics():
    def __init__(self,joint_list,robot):
        self.joints = dict()
        for j in joint_list:
            self.joints[j.name] = j
            if j.parent:
                j.parent.children.append(j)
        self.base = self.joints[joint_list[0].name]
        self.robot = robot
        robot.kine = self
        self.get_pose()

    def joint_to_base(self,joint):
        if isinstance(joint,str):
            joint = self.joints[joint]
        Tinv = geometry.identity()
        j = joint
        while j is not self.base and j.parent is not None:
            Tinv = j.parent.this_link_to_this_joint().dot(
                j.this_joint_to_parent_link.dot(Tinv)
                )
            j = j.parent
        if j:
            return Tinv
        else:
            raise Exception('Joint %s has no path to base frame' % joint)

    def base_to_joint(self,joint):
        return np.linalg.inv(self.joint_to_base(joint))

    def joint_to_joint(self,joint1,joint2):
        return self.base_to_joint(joint2).dot(self.joint_to_base(joint1))

    def link_to_base(self,joint):
        if isinstance(joint,str):
            joint = self.joints[joint]
        return self.joint_to_base(joint).dot(joint.this_link_to_this_joint())

    def base_to_link(self,joint):
        return np.linalg.inv(self.link_to_base(joint))

    def link_to_link(self,joint1,joint2):
        return self.base_to_link(joint2).dot(self.link_to_base(joint1))

    def get_pose(self):
        for j in self.joints.values():
            j.q = j.getter()
