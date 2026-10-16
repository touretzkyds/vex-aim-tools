from vex import *

robot = Robot()

def attitude_test(pause=False):
    while True:
        s = robot.status['robot']
        gyro = s['gyro_rate']
        accel = s['acceleration']
        print(f"gyro: {float(gyro['x']):6.1f} {float(gyro['y']):6.1f} {float(gyro['z']):6.1f}" + \
              f"   accel: {float(accel['x']):4.1f} {float(accel['y']):4.1f} {float(accel['z']):4.1f}" + \
              f"   pitch: {float(s['pitch']):5.1f} roll: {float(s['roll']):5.1f}", end='')
        if pause:
            input('   ...')
        else:
            print()

