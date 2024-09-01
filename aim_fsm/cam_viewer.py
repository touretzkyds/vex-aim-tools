"""
OpenGL-Based Camera Viewer
"""

import numpy as np

try:
    import cv2
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    pass

from . import opengl

# For capturing images
global snapno, path, running_fsm
snapno = 0
path = 'snap/'

WINDOW = None

class CamViewer():
    def __init__(self, robot, width=640, height=480,
                 windowName="Robot View"):
        self.robot = robot
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.windowName = windowName
        self.scale = 1
        self.show_axes = True
        self.crosshairs = False

    def process_image(self):
        raw = self.robot.camera_image

        """
        # Aruco image processing
        if running_fsm.aruco:
            running_fsm.robot.world.aruco.process_image(gray)
        # Other image processors can run here if the user supplies them.
        running_fsm.user_image(curim,gray)
        # Done with image processing

        # Annotate and display image if requested
        if running_fsm.force_annotation or running_fsm.cam_viewer is not None:
            # Apply Cozmo SDK annotations and rescale.
            if running_fsm.annotate_sdk:
                coz_ann = self.robot.world.latest_image.annotate_image(scale=scale)
                annotated_im = np.array(coz_ann)
            elif scale != 1:
                shape = curim.shape
                dsize = (scale*shape[1], scale*shape[0])
                annotated_im = cv2.resize(curim, dsize)
            else:
                annotated_im = curim
            # Aruco annotation
            if running_fsm.aruco and \
                   len(running_fsm.robot.world.aruco.seen_marker_ids) > 0:
                annotated_im = running_fsm.robot.world.aruco.annotate(annotated_im,scale)
            # Other annotators can run here if the user supplies them.
            annotated_im = running_fsm.user_annotate(annotated_im)
            # Done with annotation
            # Yellow viewer crosshairs
            if running_fsm.viewer_crosshairs:
                shape = annotated_im.shape
                cv2.line(annotated_im, (int(shape[1]/2),0), (int(shape[1]/2),shape[0]), (0,255,255), 1)
                cv2.line(annotated_im, (0,int(shape[0]/2)), (shape[1],int(shape[0]/2)), (0,255,255), 1)
            image = annotated_im
        """
        scale = 1
        if scale == 1:
            image = raw.copy()
        else:
            shape = raw.shape
            dsize = (scale*shape[1], scale*shape[0])
            image = cv2.resize(raw, dsize)

        if self.crosshairs:
            cv2.line(image, (int(self.width/2), 0), (int(self.width/2), self.height), (0,255,255), 1)
            cv2.line(image, (0, int(self.height/2)), (self.width, int(self.height/2)), (0,255,255), 1)

        for obj in self.robot.robot0.status['aivision']['objects']['items']:
            name = obj['name']
            if name == 'Ball':
                color = (0, 255, 255)
            elif name == 'OrangeBarrel':
                color = (50, 100, 255)
            elif name == 'BlueBarrel':
                color = (255, 50, 50)
            elif name == 'Robot':
                color = (0, 0, 0)
            else:
                color = (0, 255, 0)
            cv2.rectangle(image,
                          (obj['originx']*2, obj['originy']*2),
                          ((obj['originx'] + obj['width'])*2, (obj['originy'] + obj['height'])*2),
                          color,
                          1) 
        if self.robot.aruco and len(self.robot.aruco.seen_marker_ids) > 0:
            self.robot.aruco.annotate(image,scale)
        self.robot.annotated_image = image.copy()
        # Done with annotation
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height, 0, GL_BGR, GL_UNSIGNED_BYTE, image)
        glutPostRedisplay()

    # ================ Window Setup ================
    def window_creator(self):
        global WINDOW
        #glutInit(sys.argv)
        WINDOW = opengl.create_window(
            bytes(self.windowName, 'utf-8'), (self.width, self.height))
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.width, self.height)
        glutInitWindowPosition(100, 100)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyPressed)
        glutSpecialFunc(self.specialKeyPressed)
        glutSpecialUpFunc(self.specialKeyUp)

    def start(self):  # Displays in background
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)

    def display(self):
        self.process_image()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_TEXTURE_2D)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

        # Set Projection Matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(0, self.width, 0, self.height)

        glMatrixMode(GL_TEXTURE)
        glLoadIdentity()
        glScalef(1.0, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0)
        glVertex2f(0.0, 0.0)
        glTexCoord2f(1.0, 0.0)
        glVertex2f(self.width, 0.0)
        glTexCoord2f(1.0, 1.0)
        glVertex2f(self.width, self.height)
        glTexCoord2f(0.0, 1.0)
        glVertex2f(0.0, self.height)
        glEnd()

        glFlush()
        glutSwapBuffers()

    def reshape(self, w, h):
        if h == 0:
            h = 1

        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)

        glLoadIdentity()
        nRange = 1.0
        if w <= h:
            glOrtho(-nRange, nRange, -nRange*h/w, nRange*h/w, -nRange, nRange)
        else:
            glOrtho(-nRange*w/h, nRange*w/h, -nRange, nRange, -nRange, nRange)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def keyPressed(self, key, x, y):
        if ord(key) == 27:
            print("Use 'exit' to quit.")
            return
        if key == b'c':
            print("Taking a raw snap")
            self.capture_raw()
        if key == b'C':
            print("Taking an annotated snap")
            self.capture_annotated()
        self.display()

    def specialKeyPressed(self, key, x, y):
        global leftorrightindicate, globthres
        if key == GLUT_KEY_LEFT:
            #self.robot.drive_wheels(-100, 100)
            leftorrightindicate = True
            globthres=100
        elif key == GLUT_KEY_RIGHT:
            #self.robot.drive_wheels(100, -100)
            leftorrightindicate = True
            globthres = 100
        elif key == GLUT_KEY_UP:
            #self.robot.drive_wheels(200, 200)
            leftorrightindicate = False
            globthres = 100
        elif key == GLUT_KEY_DOWN:
            #self.robot.drive_wheels(-200, -200)
            leftorrightindicate = True
            globthres = 100
        glutPostRedisplay()

    def specialKeyUp(self, key, x, y):
        global leftorrightindicate, go_forward
        #self.robot.drive_wheels(0, 0)
        leftorrightindicate = True
        go_forward = GLUT_KEY_UP
        glutPostRedisplay()

    def capture_raw(self, name='robot_snap'):
        self.capture_image(self.robot.camera_image, name)

    def capture_annotated(self, name='robot_asnap'):
        self.capture_image(self.robot.annotated_image, name)

    def capture_image(self, image, name):
        global snapno, path
        if not os.path.exists(path):
                os.makedirs(path)
        filename = f"{path}{name}{snapno}.png"
        res = cv2.imwrite(filename, image)
        print(f"Wrote {filename} with result {res}")
        snapno +=1
