# -*- coding: utf-8 -*-

import numpy as np
from scipy.stats import norm
import math
import matplotlib.pyplot as plt


def beam_circle_intersection(A, B, C, r):
    d_0 = np.abs(C)
    if d_0 > r:
        return np.array([])
    x_0 = -A*C
    y_0 = -B*C
    if math.isclose(d_0, r):
        return np.array([[x_0, y_0]])
    d = np.sqrt(r*r - C*C)
    x_1 = x_0 + B*d
    y_1 = y_0 - A*d
    x_2 = x_0 - B*d
    y_2 = y_0 + A*d
    return np.array([[x_1, y_1], [x_2, y_2]])


"""
returns the distance to the closest intersection between a beam and an array of circles
the beam starts at (x,y) and has the angle theta (rad) to the x-axis
circles is a numpy array with the structure [circle1, circle2,...]
where each element is a numpy array [x_c, y_c, r] describing a circle 
with the center (x_c, y_c) and radius r
"""
def distance_to_closest_intersection(x, y, theta, circles):
    beam_dir_x = np.cos(theta)
    beam_dir_y = np.sin(theta)
    min_dist = float('inf')
    # iterate over all circles
    for circle in circles:
        # compute the line equation parameters for the beam
        # in a shifted coordinate system, with the circle center at origin
        x_shifted = x - circle[0]
        y_shifted = y - circle[1]
        # vector (A,B) is a normal vector orthogonal to the beam
        A = beam_dir_y
        B = -beam_dir_x
        C = -(A*x_shifted + B*y_shifted)
        intersections = beam_circle_intersection(A, B, C, circle[2])
        for isec in intersections:
            # check if intersection is in front of the robot
            dot_prod = (isec[0]-x_shifted)*beam_dir_x + (isec[1]-y_shifted)*beam_dir_y
            if dot_prod > 0.0:
                dist = np.sqrt(np.square(isec[0]-x_shifted)+np.square(isec[1]-y_shifted))
                if dist < min_dist:
                    min_dist = dist

    return min_dist


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
    prob_scan = 1.0
    for i in range(z_scan.size):
        if z_scan[i] > z_max:
            continue
        eta = normalizer(z_scan_exp[i], b, z_max)
        prob_z = (eta/np.sqrt(2.0*np.pi*b))*np.exp(-0.5*np.square(z_scan[i]-z_scan_exp[i])/b)
        prob_scan *= prob_z
    return prob_scan


def main():
    # define the circles in the map
    circles = np.array([[3.0, 0.0, 0.5], [4.0, 1.0, 0.8], [5.0, 0.0, 0.5], [0.7, -1.3, 0.5]])
    # robot pose
    pose = np.array([1.0, 0.0, 0.0])
    beam_directions = np.linspace(-np.pi/2, np.pi/2, 21)
    # load measurements
    z_scan = np.load('z_scan.npy')

    # compute the expected ranges using the intersection function
    # if you are not able to make it work, comment out the following three lines and load the values from file
    z_scan_exp = np.zeros(beam_directions.shape)
    for i in range(beam_directions.size):
        z_scan_exp[i] = distance_to_closest_intersection(pose[0], pose[1], beam_directions[i], circles)

    #z_scan_exp = np.load('z_scan_exp.npy')

    z_max = 10.0
    b = 1.0
    # compute the scan probability using the beam-based model
    print("the scan probability is ", beam_based_model(z_scan*100.0, z_scan_exp*100.0, b, z_max*100.0))

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
        circle_plot = plt.Circle((circle[0], circle[1]), radius=circle[2], color='black', zorder=1)
        axes.add_patch(circle_plot)


    #noise = np.random.normal(0.0, b, z_scan_exp.size)
    #z_scan = z_scan_exp + noise/100.0

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
    #plt.savefig('circle_world.png')
    plt.show()
    #np.save('z_scan_exp', z_scan_exp)
    #np.save('z_scan', z_scan)


if __name__ == "__main__":
    main()