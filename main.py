import asyncio
import threading

from aim_fsm import *

global loop
global robot_for_loading

loop = asyncio.get_event_loop()
loop.set_debug(True)

def loopthread():
    global loop
    print(f"loop thead in {threading.current_thread().native_id}")
    loop.run_forever()


print(f"main thead in {threading.current_thread().native_id}")

th = threading.Thread(target=loopthread)
th.daemon = True
print("Starting")
th.start()
print("Started")
robot = Robot(loop=loop)
robot.cam_viewer = CamViewer(robot)
robot.cam_viewer.start()

evbase.robot_for_loading = robot

tracefsm(9)

def fwd50():
    print(robot.status)
    robot.robot0.drive(0,50)
    time.sleep(2)
    robot.robot0.stop_drive()
    print(robot.status)
