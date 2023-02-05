# -*- coding: utf-8 -*-

import numpy as np
from scipy.stats import norm
import math
import matplotlib.pyplot as plt

"""
returns the distance to the closest intersection between a beam and an array of circles
the beam starts at (x,y) and has the angle theta (rad) to the x-axis
circles is a numpy array with the structure [circle1, circle2,...]
where each element is a numpy array [x_c, y_c, r] describing a circle 
with the center (x_c, y_c) and radius r
"""


def distance_to_closest_intersection(x, y, theta, circles):
    # your implementation goes here
    A = math.sin(theta)
    B = -math.cos(theta)
    dis = []
    for circle in circles:
        xc, yc, r = circle[:]
        C = A*(xc-x) + B*(yc-y)
        d0 = abs(C)
        x0 = - A*C / (A**2+B**2)
        y0 = - B*C / (A**2+B**2)

        if d0 <= r:
            d = np.sqrt(r**2 - (C**2 / (A**2+B**2)))
            m = np.sqrt(d**2 / (A**2+B**2))
            xa = x0 + B*m
            ya = y0 - A*m
            if xc-x+xa > 0:
                dis.append(np.linalg.norm([xc-x+xa, yc-y+ya]))

            xb = x0 - B*m
            yb = y0 + A*m
            if xc-x+xb > 0:
                dis.append(np.linalg.norm([xc-x+xb, yc-y+yb]))

    if len(dis) == 0:
        # no point
        dis.append(float('inf'))

    return np.min(dis)


"""
returns the normalizer value in the hit-probability function
z_exp is the expected range (in cm)
b the variance
z_max is the maximum range (in cm) 
"""


def normalizer(z_exp, b, z_max):
    std_dev = np.sqrt(b)
    return 1.0/(norm(z_exp, std_dev).cdf(z_max) - norm(z_exp, std_dev).cdf(0.0))


"""
z_scan and z_scan_exp are numpy arrays containing the measured and expected range values (in cm)
b is the variance parameter of the measurement noise
z_max is the maximum range (in cm)
returns the probability of the scan according to the simplified beam-based model
"""


def beam_based_model(z_scan, z_scan_exp, b, z_max):

    # convert inf as z_max to make sure no nan-type error in normalizer
    z_scan_exp = np.array([z_max if math.isinf(x) else x for x in z_scan_exp])
    z_scan = np.array([z_max if math.isinf(x) else x for x in z_scan])
    Idx_inf = np.where(z_scan_exp == z_max)

    res = normalizer(z_scan_exp, b, z_max) * (1 / np.sqrt(2*np.pi*b)
                                              ) * np.exp(-(z_scan - z_scan_exp)**2 / (2*b))

    # convert nan as probability of 1
    res[Idx_inf] = 1

    return res


def main():
    # define the circles in the map
    circles = np.array([[3.0, 0.0, 0.5], [4.0, 1.0, 0.8],
                       [5.0, 0.0, 0.5], [0.7, -1.3, 0.5]])
    # robot pose
    pose = np.array([1.0, 0.0, 0.0])
    beam_directions = np.linspace(-np.pi/2, np.pi/2, 21)
    # load measurements
    z_scan = np.load('z_scan.npy')

    # compute the expected ranges using the intersection function
    # if you are not able to make it work, comment out the following three lines and load the values from file
    z_scan_exp = np.zeros(beam_directions.shape)
    # for i in range(beam_directions.size):
    # z_scan_exp[i] = distance_to_closest_intersection(
    # pose[0], pose[1], beam_directions[i], circles)

    z_scan_exp = np.load('z_scan_exp.npy')

    z_max = 10.0
    b = 1.0
    # compute the scan probability using the beam-based model
    # *100.0 is conversion from meters to centimeters
    print("the scan probability is ", beam_based_model(
        z_scan*100.0, z_scan_exp*100.0, b, z_max*100.0))

    ########### visualization #################################
    plt.axes().set_aspect('equal')
    plt.xlim([-0, 6])
    plt.ylim([-2, 2])
    plt.plot(pose[0], pose[1], "bo")

    fig = plt.gcf()
    axes = fig.gca()
    for i in range(beam_directions.size):
        theta = beam_directions[i]
        x_points = [pose[0], pose[0] + 10*np.cos(theta)]
        y_points = [pose[1], pose[1] + 10*np.sin(theta)]
        plt.plot(x_points, y_points, linestyle='dashed', color='red', zorder=0)

    for circle in circles:
        circle_plot = plt.Circle(
            (circle[0], circle[1]), radius=circle[2], color='black', zorder=1)
        axes.add_patch(circle_plot)

    for i in range(beam_directions.size):
        if z_scan_exp[i] > z_max:
            continue
        theta = beam_directions[i]
        hit_x = pose[0] + np.cos(theta) * z_scan_exp[i]
        hit_y = pose[1] + np.sin(theta) * z_scan_exp[i]
        plt.plot(hit_x, hit_y, "ro")
        #meas_x = pose[0] + np.cos(theta) * z_scan[i]
        #meas_y = pose[1] + np.sin(theta) * z_scan[i]
        #plt.plot(meas_x, meas_y, "go")

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")

    plt.show()


if __name__ == "__main__":
    main()
