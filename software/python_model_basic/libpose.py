import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2

import funs

def eq(e, y, l1, l2):
    acc = 0
    for row in y:
        roww = np.array([row])
        acc += funs.eqa(e, roww, l1, l2)

    acc += funs.eqb(e, None, l1, l2) + funs.eqc(e, None, l1, l2)

    return acc

def eq_de(e, y, l1, l2):
    acc = np.zeros((3, 3))
    for row in y:
        roww = np.array([row])
        rf = np.array([
            funs.eqa_d_e0(e, roww, l1, l2),
            funs.eqa_d_e1(e, roww, l1, l2),
            funs.eqa_d_e2(e, roww, l1, l2),
            funs.eqa_d_e3(e, roww, l1, l2),
            funs.eqa_d_e4(e, roww, l1, l2),
            funs.eqa_d_e5(e, roww, l1, l2),
            funs.eqa_d_e6(e, roww, l1, l2),
            funs.eqa_d_e7(e, roww, l1, l2),
            funs.eqa_d_e8(e, roww, l1, l2),
        ])
        rf.shape = (3, 3)
        acc += rf
    
    rf = np.array([
        funs.eqb_d_e0(e, None, l1, l2) + funs.eqc_d_e0(e, None, l1, l2),
        funs.eqb_d_e1(e, None, l1, l2) + funs.eqc_d_e1(e, None, l1, l2),
        funs.eqb_d_e2(e, None, l1, l2) + funs.eqc_d_e2(e, None, l1, l2),
        funs.eqb_d_e3(e, None, l1, l2) + funs.eqc_d_e3(e, None, l1, l2),
        funs.eqb_d_e4(e, None, l1, l2) + funs.eqc_d_e4(e, None, l1, l2),
        funs.eqb_d_e5(e, None, l1, l2) + funs.eqc_d_e5(e, None, l1, l2),
        funs.eqb_d_e6(e, None, l1, l2) + funs.eqc_d_e6(e, None, l1, l2),
        funs.eqb_d_e7(e, None, l1, l2) + funs.eqc_d_e7(e, None, l1, l2),
        funs.eqb_d_e8(e, None, l1, l2) + funs.eqc_d_e8(e, None, l1, l2),
    ])
    rf.shape = (3, 3)
    acc += rf

    return acc

def eq_dl1(e, l1):
    return funs.eqb_d_l1(e, None, l1, None)
def eq_dl2(e, l2):
    return funs.eqc_d_l2(e, None, None, l2)

# ==== Find ticks per rotation ====

# Clock frequency of timer is 16MHz
clock_freq = 16000000
clock_period = 1 / clock_freq

# Rotors rotate at 60Hz
rotor_period = 1 / 60

ticks_per_rotation_f = rotor_period / clock_period
ticks_to_center_f = ticks_per_rotation_f / 4
ticks_to_rad = (2*np.pi)/ticks_per_rotation_f

def process_points():

    # ==== Normalize ====
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

    # ==== Convert to homogenous coordinates (projective space) ====

    points_transposed = np.transpose(points)

    bsa_points = np.transpose([points_transposed[0], points_transposed[1]])
    bsb_points = np.transpose([points_transposed[2], points_transposed[3]])

    def rot_to_homog(point):
        return np.array([
                np.sin(point[1]),
                -np.sin(point[0]) * np.cos(point[1]),
                np.cos(point[0]) * np.cos(point[1])
            ])

    bsa_homog_rays = np.array(list(map(rot_to_homog, bsa_points)))
    bsb_homog_rays = np.array(list(map(rot_to_homog, bsb_points)))

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
    
    point = (D + E) / 2
    dist = np.linalg.norm(E - D)
    
    return point

def eight_point_algorithm(points1, points2):
        
    nump = len(points1)
    points1_transp = np.transpose(points1)
    points2_transp = np.transpose(points2)
    
    Y = np.array([
            points1_transp[0]*points2_transp[0], # x x'
            points1_transp[0]*points2_transp[1], # x y'
            points1_transp[0],                   # x
            points1_transp[1]*points2_transp[0], # y x'
            points1_transp[1]*points2_transp[1], # y y'
            points1_transp[1],                   # y
            points2_transp[0],                   # x'
            points2_transp[1],                   # y'
            np.ones(nump)
        ])
    
    Y_u, Y_s, Y_v = np.linalg.svd(Y)
        
    Ea = Y_u.transpose()[8].reshape((3,3))
    return Ea

def solve():
    print("CORR")
    l1 = 10
    l2 = 10
    residual = eq(Ea, Y.transpose(), l1, l2)
    print("Residual: ", residual)
    while residual > 0.001:
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
        Ea -= 0.002 * eq_de(Ea, Y.transpose(), l1, l2)
        residual = eq(Ea, Y.transpose(), l1, l2)
        print("Residual: ", residual, "L:", l1, l2)
    print("Final residual: ", residual)

    print(Ea)
    print(np.linalg.det(Ea))

    p1cv = np.transpose([points1_transp[0]/points1_transp[2], points1_transp[1]/points1_transp[2]])
    p2cv = np.transpose([points2_transp[0]/points2_transp[2], points2_transp[1]/points2_transp[2]])
    
    rp_points, rp_R, rp_t, rp_mask = cv2.recoverPose(Ea, p1cv, p2cv)
    print("points:", rp_points)
    print("rotation:", rp_R)
    print("translation:", rp_t)
    
    return (rp_t, rp_R)
    

class Ingester:
    
    def __init__(self):
        pass

    def ingest_line(self, text):
        pass


