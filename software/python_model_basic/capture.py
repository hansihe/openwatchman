import re
import time
import serial
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === Plot ===

fig = plt.figure(figsize=(5, 5), dpi=80)
ax = fig.add_subplot(111, projection='3d')
ax.set_proj_type('persp')

ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(0, 4)

bs0_arrow = ax.quiver(0, 0, 0, 0, 0, 1)
bs1_arrow = ax.quiver(0, 0, 1, 0, 0, 1)
bs1_arrow.set_segments([[[1, 0, 0], [1, 0, 1]]])

baseline_arrow = ax.quiver(0, 0, 0, 1, 0, 0, color='r')

plt.ion()
plt.show()

# ==== Find ticks per rotation ====

# Clock frequency of timer is 16MHz
clock_freq = 16000000
clock_period = 1 / clock_freq

# Rotors rotate at 60Hz
rotor_period = 1 / 60

ticks_per_rotation_f = (rotor_period / clock_period)# - 7000
ticks_to_center_f = ticks_per_rotation_f / 4
ticks_to_rad = (2*np.pi)/ticks_per_rotation_f

def rot_to_homog(x, y):
    nx = [math.cos(x), 0, -math.sin(x)]
    ny = [0, math.cos(y), -math.sin(y)]
    new_s =  np.cross(nx, ny)
    if new_s[2] < 0:
        new_s = new_s * -1
    return new_s
    #old_s = [
    #    math.sin(y),
    #    -math.sin(x) * math.cos(y),
    #    math.cos(x) * math.cos(y),
    #]

point_buffer_cursor = 0
point_buffer_filled = False
point_buffer_a = np.zeros((1000, 3))
point_buffer_b = np.zeros((1000, 3))

def eight_point_algorithm(points1, points2):
    nump = len(points1)
    points1_transp = np.transpose(points1)
    points2_transp = np.transpose(points2)

    Y = np.array([
            points1_transp[0]*points2_transp[0], # x x'
            points1_transp[0]*points2_transp[1], # x y'
            points1_transp[0]*points2_transp[2], # x
            points1_transp[1]*points2_transp[0], # y x'
            points1_transp[1]*points2_transp[1], # y y'
            points1_transp[1]*points2_transp[2], # y
            points1_transp[2]*points2_transp[0], # x'
            points1_transp[2]*points2_transp[1], # y'
            points1_transp[2]*points2_transp[2],
        ])

    Y_u, Y_s, Y_v = np.linalg.svd(Y)
    Ea_us = Y_u.transpose()[8].reshape((3, 3))

    print("Solution quality:", Y_s[8])

    p1cv = np.transpose([points1_transp[0]/points1_transp[2], points1_transp[1]/points1_transp[2]])
    p2cv = np.transpose([points2_transp[0]/points2_transp[2], points2_transp[1]/points2_transp[2]])

    rp_points, rp_R, rp_t, rp_mask = cv2.recoverPose(Ea_us, p1cv, p2cv)

    print("rotation:", rp_R)
    print("translation:", rp_t)

    rp_t_vec = rp_t.transpose()[0]
    rp_R_fw = rp_R.transpose()[2]
    rp_R_fw = rp_R_fw / np.linalg.norm(rp_R_fw)

    bs1_arrow.set_segments([[rp_t_vec, rp_t_vec + rp_R_fw]])
    baseline_arrow.set_segments([[[0, 0, 0], rp_t_vec]])

    fig.canvas.draw()
    fig.canvas.flush_events()

def try_solve():
    if point_buffer_filled:
        samples = np.arange(0, 1000, 5)
    else:
        samples = np.arange(0, point_buffer_cursor, 5)

    samples_a = point_buffer_a[samples]
    samples_b = point_buffer_b[samples]

    if len(samples_a) > 10:
        eight_point_algorithm(samples_a, samples_b)

def handle_point(meas):
    global point_buffer_cursor

    if None in meas:
        return

    # Convert to radians
    b0x = (meas[0] - ticks_to_center_f) * ticks_to_rad
    b0y = (meas[1] - ticks_to_center_f) * ticks_to_rad
    b1x = (meas[2] - ticks_to_center_f) * ticks_to_rad
    b1y = (meas[3] - ticks_to_center_f) * ticks_to_rad

    # Point to homogenous
    p0 = rot_to_homog(b0x, b0y)
    p1 = rot_to_homog(b1x, b1y)

    point_buffer_a[point_buffer_cursor] = p0
    point_buffer_b[point_buffer_cursor] = p1

    try_solve()

    point_buffer_cursor += 1
    if point_buffer_cursor == 1000:
        point_buffer_filled = True
        point_buffer_cursor = 0

with serial.Serial('/dev/ttyUSB0', 115200) as ser:

    next_state = 1
    point_parts = []

    while True:
        try:
            line = ser.readline().decode('ascii').strip()

            if line.startswith("sweep:"):
                match = re.match('^sweep: s0\(([^)]+)\) s1\(([^)]+)\) s1start\((\d+)\) sensor0\((\d+)\) state\((\d+)\)$', line)

                s0 = match.group(1)
                s1 = match.group(2)
                s1_start_time = int(match.group(3))
                time = int(match.group(4))
                state = int(match.group(5))

                if state == next_state:
                    if state == 3 or state == 4:
                        if time is not 0:
                            time = time - s1_start_time
                    if time is 0:
                        point_parts.append(None)
                    else:
                        point_parts.append(time)
                    if state == 4:
                        next_state = 1
                        assert(len(point_parts) == 4)
                        handle_point(point_parts)
                        point_parts = []
                    else:
                        next_state += 1
                else:
                    point_parts = []
                    next_state = 1

        except UnicodeDecodeError:
            pass
