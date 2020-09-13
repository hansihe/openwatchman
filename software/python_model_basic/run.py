import re
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2

import funs

# ==== Read samples ====
samples = []
with open("movements_points.txt", 'rt') as f:
    for line in f:
        match = re.match('^sweep: s0\(([^)]+)\) s1\(([^)]+)\) s1start\((\d+)\) sensor0\((\d+)\)$', line.strip())
        s0 = match.group(1)
        s1 = match.group(2)
        s1_start_time = int(match.group(3))
        time = int(match.group(4))
        
        if "s0" in s0:
            station = 0
        else:
            station = 1
            time = time - s1_start_time
        
        if "a0" in s0:
            axis = 0
        else:
            axis = 1
            
        samples.append(((station, axis), time))

# ==== Find ticks per rotation ====

# Clock frequency of timer is 16MHz
clock_freq = 16000000
clock_period = 1 / clock_freq

# Rotors rotate at 60Hz
rotor_period = 1 / 60

ticks_per_rotation_f = (rotor_period / clock_period)# - 7000
ticks_to_center_f = ticks_per_rotation_f / 4
ticks_to_rad = (2*np.pi)/ticks_per_rotation_f

# ==== Normalize points ====

points = []

pos = 0
while len(samples) > pos+4:
    vals = [samples[pos], samples[pos+1], samples[pos+2], samples[pos+3]]
    pos += 4
    
    raw_vals = list(map(lambda s: s[1], vals))
    
    assert(vals[0][0][0] == 0)
    assert(vals[0][0][1] == 0)

    for val in raw_vals:
        if val < 0:
            continue
    
    points.append((
            (vals[0][1]-ticks_to_center_f)*ticks_to_rad,
            (vals[1][1]-ticks_to_center_f)*ticks_to_rad,
            (vals[2][1]-ticks_to_center_f)*ticks_to_rad,
            (vals[3][1]-ticks_to_center_f)*ticks_to_rad
        ))

# ==== Native lib for derivative functions ====

#from ctypes import cdll
#import ctypes
#lib = cdll.LoadLibrary('./libessential_matrix_eqs.so')
#
#e_mat = ctypes.c_double * 9
#y_mat = ctypes.c_double * 9
#l1_typ = ctypes.c_double
#l2_typ = ctypes.c_double
#
##lib.eqa.restype = ctypes.c_double
#
#funs = [
#    ('eqa', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e0', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e1', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e2', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e3', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e4', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e5', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e6', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e7', [e_mat, y_mat], ctypes.c_double),
#    ('eqa_d_e8', [e_mat, y_mat], ctypes.c_double),
#]
#for (name, args, restype) in funs:
#    fun = lib[name]
#    fun.argtypes = args
#    fun.restype = restype
#print(lib.eqa_d_e0.argtypes, lib.eqa_d_e0.restype)
#print(lib.eqa_d_e8.argtypes, lib.eqa_d_e8.restype)
#
#def eqa_e(e, y, l1, l2):
#    c_e = e_mat(*list(e.flat))
#    acc = 0
#    for row in y:
#        c_y = y_mat(*list(row.flat))
#        acc += lib.eqa(c_e, c_y)
#    return acc
#def eqa_d(e, y):
#    c_e = e_mat(*list(e.flat))
#    acc = np.zeros((3, 3))
#    for row in y:
#        c_y = y_mat(*list(row.flat))
#        rf = np.array([
#            lib.eqa_d_e0(c_e, c_y),
#            lib.eqa_d_e1(c_e, c_y),
#            lib.eqa_d_e2(c_e, c_y),
#            lib.eqa_d_e3(c_e, c_y),
#            lib.eqa_d_e4(c_e, c_y),
#            lib.eqa_d_e5(c_e, c_y),
#            lib.eqa_d_e6(c_e, c_y),
#            lib.eqa_d_e7(c_e, c_y),
#            lib.eqa_d_e8(c_e, c_y),
#        ])
#        print(list(c_e), list(c_y), lib.eqa_d_e8(c_e, c_y))
#        rf.shape = (3, 3)
#        #print(rf)
#        acc += rf
#    return acc
#
#eqa = lib.eqa
#eqa.restype = ctypes.c_double
#
#eqa_d_e0 = lib.eqa_d_e0
#eqa_d_e0.restype = ctypes.c_double
#
#print("foo", eqa_d(np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]), np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1]])))

def eq(e, y, penalty):
    acc = 0
    for row in y:
        roww = np.array([row])
        acc += funs.eqa(e, roww, penalty, penalty, penalty)

    acc += funs.eqb(e, None, penalty, penalty, penalty) + funs.eqc(e, None, penalty, penalty, penalty) #+ funs.eqd(e, None, penalty, penalty, penalty)

    return acc

def eq_de(e, y, penalty):
    acc = np.zeros((3, 3))
    for row in y:
        roww = np.array([row])
        rf = np.array([
            funs.eqa_d_e0(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e1(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e2(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e3(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e4(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e5(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e6(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e7(e, roww, penalty, penalty, penalty),
            funs.eqa_d_e8(e, roww, penalty, penalty, penalty),
        ])
        rf.shape = (3, 3)
        acc += rf
    
    rf = np.array([
        funs.eqb_d_e0(e, None, penalty, penalty, penalty) + funs.eqc_d_e0(e, None, penalty, penalty, penalty), # + funs.eqd_d_e0(e, None, penalty, penalty, penalty),
        funs.eqb_d_e1(e, None, penalty, penalty, penalty) + funs.eqc_d_e1(e, None, penalty, penalty, penalty), # + funs.eqd_d_e1(e, None, penalty, penalty, penalty),
        funs.eqb_d_e2(e, None, penalty, penalty, penalty) + funs.eqc_d_e2(e, None, penalty, penalty, penalty), # + funs.eqd_d_e2(e, None, penalty, penalty, penalty),
        funs.eqb_d_e3(e, None, penalty, penalty, penalty) + funs.eqc_d_e3(e, None, penalty, penalty, penalty), # + funs.eqd_d_e3(e, None, penalty, penalty, penalty),
        funs.eqb_d_e4(e, None, penalty, penalty, penalty) + funs.eqc_d_e4(e, None, penalty, penalty, penalty), # + funs.eqd_d_e4(e, None, penalty, penalty, penalty),
        funs.eqb_d_e5(e, None, penalty, penalty, penalty) + funs.eqc_d_e5(e, None, penalty, penalty, penalty), # + funs.eqd_d_e5(e, None, penalty, penalty, penalty),
        funs.eqb_d_e6(e, None, penalty, penalty, penalty) + funs.eqc_d_e6(e, None, penalty, penalty, penalty), # + funs.eqd_d_e6(e, None, penalty, penalty, penalty),
        funs.eqb_d_e7(e, None, penalty, penalty, penalty) + funs.eqc_d_e7(e, None, penalty, penalty, penalty), # + funs.eqd_d_e7(e, None, penalty, penalty, penalty),
        funs.eqb_d_e8(e, None, penalty, penalty, penalty) + funs.eqc_d_e8(e, None, penalty, penalty, penalty), # + funs.eqd_d_e8(e, None, penalty, penalty, penalty),
    ])
    rf.shape = (3, 3)
    acc += rf

    return acc

#def eq_dl1(e, l1):
#    return funs.eqb_d_l1(e, None, l1, None)
#def eq_dl2(e, l2):
#    return funs.eqc_d_l2(e, None, None, l2)

#print("foo", eqa_d(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[1, 1, 1.5, 1, 1, 1, 1, 1, 1]]), 1, 1))

# ==== Convert to homogenous coordinates (projective space) ====

points_transposed = np.transpose(points)

bsa_points = np.transpose([points_transposed[0], points_transposed[1]])
bsb_points = np.transpose([points_transposed[2], points_transposed[3]])

def rot_to_homog(point):
    nx = [math.cos(point[0]), 0, -math.sin(point[0])]
    ny = [0, math.cos(point[1]), -math.sin(point[1])]
    new_s =  np.cross(nx, ny)
    if new_s[2] < 0:
        new_s = new_s * -1
    return new_s

#def rot_to_homog(point):
#    return np.array([
#            np.sin(point[1]),
#            -np.sin(point[0]) * np.cos(point[1]),
#            np.cos(point[0]) * np.cos(point[1])
#        ])

bsa_homog_rays = np.array(list(map(rot_to_homog, bsa_points)))
bsb_homog_rays = np.array(list(map(rot_to_homog, bsb_points)))

# ==== Implementation ====

fig = plt.figure(figsize=(5, 5), dpi=80)
ax = fig.add_subplot(111, projection='3d')
ax.set_proj_type('persp')

ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(0, 4)

def ray_ray_intersection(translation, rotation, pointa, pointb):
    
    pointa_start = np.array([0, 0, 0])
    pointa_dir = pointa
    
    pointb_start = translation.transpose()[0]
    pointb_dir = np.matmul(rotation, np.transpose([pointb])).transpose()[0]
    
    c = translation.transpose()[0]
    
    a_dot_b = np.dot(pointa_dir, pointb_dir)
    a_dot_c = np.dot(pointa_dir, c)
    b_dot_b = np.dot(pointb_dir, pointb_dir)
    a_dot_a = np.dot(pointa_dir, pointa_dir)
    b_dot_c = np.dot(pointb_dir, c)

    D = pointa_start + pointa_dir * ((-a_dot_b*b_dot_c +a_dot_c*b_dot_b)/(a_dot_a*b_dot_b -a_dot_b*a_dot_b))
    E = pointb_start + pointb_dir * ((a_dot_b*a_dot_c -b_dot_c*a_dot_a)/(a_dot_a*b_dot_b -a_dot_b*a_dot_b))
    
    #print(b_dot_c)
    #print(((-a_dot_b*b_dot_c +a_dot_c*b_dot_b)/(a_dot_a*b_dot_b -a_dot_b*a_dot_b)))
    
    point = (D + E) / 2
    dist = np.linalg.norm(E - D)
    
    #print("dist: ", dist)
    #print("point: ", point)
    
    return (point, dist)
    
def eight_point_algorithm(points1, points2):
        
    nump = len(points1)
    points1_transp = np.transpose(points1)
    points2_transp = np.transpose(points2)
    
    Y = np.array([
            points1_transp[0]*points2_transp[0], # x x'
            points1_transp[0]*points2_transp[1], # x y'
            points1_transp[0]*points2_transp[2],                   # x
            points1_transp[1]*points2_transp[0], # y x'
            points1_transp[1]*points2_transp[1], # y y'
            points1_transp[1]*points2_transp[2],                   # y
            points1_transp[2]*points2_transp[0],                   # x'
            points1_transp[2]*points2_transp[1],                   # y'
            points1_transp[2]*points2_transp[2],
        ])
    
    Y_u, Y_s, Y_v = np.linalg.svd(Y)
        
    Ea_us = Y_u.transpose()[8].reshape((3,3))

    Ea_us_u, Ea_us_s, Ea_us_v = np.linalg.svd(Ea_us)
    print("SVD values:", Ea_us_s)

    #Ea = np.matmul(Ea_us_u, np.matmul(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]]), Ea_us_v))
    Ea = Ea_us

    print("fro")
    print(np.linalg.norm(Ea, 'fro'))
    print(np.diag(np.matmul(Ea.transpose(), Ea)).sum())

    print("Before:", Ea_us)
    print("After:", Ea)

    print("CORR")
    penalty = 1
    residual = eq(Ea, Y.transpose(), penalty)
    print("Residual: ", residual)
    for i in range(100):
        #if penalty > 0.01:
        #    break

        f_res = eq(Ea, Y.transpose(), penalty)
        fd_res = eq_de(Ea, Y.transpose(), penalty)
        fd_res_norm = np.linalg.norm(fd_res)

        # = START Backtracking line search =
        B = 0.8
        t = 1
        while eq(Ea - t*fd_res, Y.transpose(), penalty) > (f_res - (t/2)*fd_res_norm**2):
            t = t * B
        Ea -= t*fd_res
        # = END Backtracking line search =

        #print(np.linalg.norm(Ea, 'fro'))
        #Ea = Ea / np.linalg.norm(Ea, 'fro')

        #J_eq_de = eq_de(Ea, Y.transpose(), l1, l2).reshape(9)
        #J_eq_dl1 = eq_dl1(Ea, l1)
        #J_eq_dl2 = eq_dl2(Ea, l2)
        #J = np.array([np.ravel([J_eq_de, J_eq_dl1, J_eq_dl2])[0]])

        #J_pinv = np.linalg.pinv(J).transpose()[0]
        #print(J_pinv)
        #J_e = J_pinv[0:9].reshape((3, 3))
        #J_l1= J_pinv[9]
        #J_l2= J_pinv[10]

        #Ea += J_e * residual
        #l1 += J_l1 * residual
        #l2 += J_l2 * residual

        #H_est = np.matmul(J.transpose(), J)
        #print(H_est)

        #l1 -= 0.001 * eq_dl1(Ea, l1)
        #l2 -= 0.001 * eq_dl2(Ea, l2)


        #Ea -= 0.001 * eq_de(Ea, Y.transpose(), penalty)

        residual = eq(Ea, Y.transpose(), penalty)
        #print("Eqd residual:", funs.eqd(Ea, None, penalty, penalty, penalty))
        #penalty += 0.1
        print("Residual: ", residual, "Penalty:", penalty)

    print("Final residual: ", residual)

    print(Ea)
    print(np.linalg.det(Ea))

    #print("WOO", eqa_d(Ea, Y.transpose()))
    
    p1cv = np.transpose([points1_transp[0]/points1_transp[2], points1_transp[1]/points1_transp[2]])
    p2cv = np.transpose([points2_transp[0]/points2_transp[2], points2_transp[1]/points2_transp[2]])
    
    rp_points, rp_R, rp_t, rp_mask = cv2.recoverPose(Ea, p1cv, p2cv)
    print("points:", rp_points)
    print("rotation:", rp_R)
    print("translation:", rp_t)
    
    return (rp_t, rp_R)
    
    #p1
    
    #(r1, r2, r3) = rp_R[0,1], rp_R[1,1], rp_r[2,1]
    #x3 = ((r1 - ))
    
    #Ea_u, Ea_s, Ea_v = np.linalg.svd(Ea)
    
    #display(Ea_s)
    #E = np.matmul(Ea_u, np.matmul(np.diag([1, 1, 0]), Ea_v))
    
    #display(E)    
    
#samples = [10, 40, 70, 100, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200]
samples = np.arange(4, 1000, 10)
translation, rotation = eight_point_algorithm(bsa_homog_rays[samples], bsb_homog_rays[samples])
rotation_inv = np.linalg.inv(rotation)

ax.quiver(0, 0, 0, translation[0], translation[1], translation[2], color='r')
ax.quiver(0, 0, 0, 0, 0, 1)
ax.quiver(translation[0], translation[1], translation[2], rotation_inv[2,0], rotation_inv[2,1], rotation_inv[2,2])

dest_points = np.zeros((1000, 3))
points_error = np.zeros(1000)
for (idx, (pa, pb)) in enumerate(zip(bsa_homog_rays[:1000], bsb_homog_rays[:1000])):
    #print("before:", pb, "after:", np.matmul(rotation, pb))
    pb_r = np.matmul(rotation, pb)
    (dest_points[idx], points_error[idx]) = ray_ray_intersection(translation, rotation, pa, pb_r)
print("error ptp:", np.ptp(points_error), "min:", np.ndarray.min(points_error), "max:", np.ndarray.max(points_error))

dest_points_transp = dest_points.transpose()
ax.scatter(dest_points_transp[0], dest_points_transp[1], dest_points_transp[2], c=points_error, cmap='cool')

#display(ahhhh(bsa_homog_rays, bsb_homog_rays))

plt.show()

