import asyncio
import threading

import aim_fsm
from aim_fsm import *

print(f"main thead in {threading.current_thread().native_id}")

global loop
loop = asyncio.get_event_loop()
loop.set_debug(True)

def loopthread():
    global loop
    print(f"loop thead in {threading.current_thread().native_id}")
    loop.run_forever()

th = threading.Thread(target=loopthread)
th.daemon = True
th.start()

global robot
global robot_for_loading

robot_ip_addr = "192.168.4.1"
robot_ip_addr = "172.26.167.23"

robot = Robot(loop=loop, ip=robot_ip_addr)

evbase.robot_for_loading = robot
program.robot_for_loading = robot

tracefsm(9)

robot.loop.call_soon_threadsafe(StateMachineProgram().start)

def ptgtest(cx,cy):
    camera_res = robot.camera.resolution
    print(f"camera_res = {camera_res}")

    half_camera_max = max(*camera_res) / 2
    print(f"half_camera_max = {half_camera_max}")

    # Convert to generalized coordinates in range [-1, 1]
    center = robot.camera.center
    print(f"center = {center}")
    
    gx = (cx - center[0]) / half_camera_max
    gy = (cy - center[1]) / half_camera_max
    print(f"gx = {gx:.2f}  gy = {gy:.2f}")

    focal_length = robot.camera.focal_length
    print(f"focal_length = {focal_length}")
    
    rx = gx / (focal_length[0] / half_camera_max)
    ry = gy / (focal_length[1] / half_camera_max)
    print(f"rx = {rx}  ry = {ry}")

    ray = point(rx,ry,1)
    print("ray =")
    tprint(ray)

    cam_to_base = robot.kine.joint_to_base('camera')
    print("cam_to_base =")
    tprint(cam_to_base)
    
    offset = translation_part(cam_to_base)
    print(f"offset =")
    tprint(offset)

    rot_ray = rotation_part(cam_to_base).dot(ray)
    print("rot_ray =")
    tprint(rot_ray)

    dist = - offset[2,0]
    align = rot_ray[2,0]
    print(f"dist = {dist}    align = {align}")

    s = dist / align
    print(f"s = {s}")
    
    hit = point(rot_ray[0,0]*s, rot_ray[1,0]*s, rot_ray[2,0]*s) + offset
    print("hit = ")
    tprint(hit)

