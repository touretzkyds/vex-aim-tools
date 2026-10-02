try: import cv2
except: pass

import math
from math import pi

import numpy as np
from numpy import sqrt, arctan2

from . import camera
from .geometry import *

ARUCO_MARKER_SIZE = 25

class ArucoMarker(object):
    def __init__(self, aruco_parent, marker_id, corners, tvec, rvec):
        self.aruco_parent = aruco_parent
        self.marker_id = marker_id
        self.corners = corners

        # OpenCV Pose information
        self.tvec = tvec
        self.rvec = rvec

        # Marker coordinates in robot's camera reference frame:
        # x points right, y points down, z is depth
        self.camera_coords = (-tvec[0][0], -tvec[1][0], tvec[2][0])

        # Distance along the ground plane; particle filter ignores height so don't include camera y
        self.camera_distance = math.sqrt(tvec[0][0]**2 + tvec[2][0]**2)

        # Conversion to euler angles
        rotationm, jacob = cv2.Rodrigues(rvec)
        self.euler_angles = rotation_matrix_to_euler_angles(rotationm)

    def __str__(self):
        return "<ArucoMarker id=%d tvec=(%d,%d,%d) rvec=(%d,%d,%d) euler=(%d,%d,%d)>" % \
                (self.marker_id, *(self.tvec[:,0].tolist()),
                 *((self.rvec[:,0]*180/pi).tolist()), *self.euler_angles*180/pi)

    def __repr__(self):
        return self.__str__()


class RobotArucoDetector(object):
    def __init__(self, robot, dictionary_name, marker_size=ARUCO_MARKER_SIZE, disabled_ids=[]):
        self.robot = robot
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_name)
        detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
        self.seen_marker_ids = []
        self.seen_marker_objects = dict()
        self.disabled_ids = disabled_ids  # disable markers with high false detection rates
        self.ids = []
        self.corners = []
        self.marker_size = marker_size #these units will be pose est units!!
        self.object_corners = np.array([
            [-marker_size / 2, marker_size / 2, 0],  # Top left
            [marker_size / 2, marker_size / 2, 0],   # Top right
            [marker_size / 2, -marker_size / 2, 0],  # Bottom right
            [-marker_size / 2, -marker_size / 2, 0]  # Bottom left
            ], dtype=np.float32)

    def process_image(self,gray):
        self.seen_marker_ids = []
        self.seen_marker_objects = dict()
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None: return

        # Estimate poses
        for i in range(len(corners)):
            image_corners = corners[i]
            if type(ids[i]) is np.ndarray:  # OpenCV 4.x
                id = int(ids[i][0])
            else: # OpenCV 5.0
                id = int(ids[i])
            try:
                success, rvec, tvec = cv2.solvePnP(self.object_corners,
                                                   image_corners,
                                                   self.robot.camera.camera_matrix,
                                                   self.robot.camera.distortion_array)
            except Exception as e:
                print(f'Aruco detector: solvePnP failed:', e)
                return
            if id in self.disabled_ids: continue
            if rvec[2][0] > math.pi/2 or rvec[2][0] < -math.pi/2:
                # can't see a marker facing away from us, so bogus
                print(f'Marker rejected! id={id}  tvec={tvec.tolist()}  ' +
                      f'rvec={(rvec*180/pi).tolist()}')
                continue
            marker = ArucoMarker(self, id, image_corners, tvec, rvec)
            self.seen_marker_ids.append(id)
            self.seen_marker_objects[id] = marker

    def annotate(self, image, scale_factor):
        scaled_corners = [ np.multiply(corner, scale_factor) for corner in self.corners ]
        displayim = cv2.aruco.drawDetectedMarkers(image, scaled_corners, self.ids)

        #add poses currently fails since image is already scaled. How to scale camMat?
        #if(self.ids is not None):
        #    for i in range(len(self.ids)):
        #      displayim = cv2.aruco.drawAxis(displayim,self.cameraMatrix,
        #                    self.distortionArray,self.rvecs[i],self.tvecs[i]*scale_factor,self.axisLength*scale_factor)
        return displayim
