"""proxy_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
MAX_SPEED = 12.3

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get wheel motor controllers
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')


leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
#deg = 0.325 * np.pi / 0.195
#leftMotor.setPosition(deg)
#rightMotor.setPosition(-deg)

# get and enable lidar
lidar = robot.getDevice('Sick LMS 291')
lidar.enable(60)


move_to_closest = True
rotation = False
follow_wall = False
threshold = 0.5
lidar_to_rob = [0.08, 0, 0.21]


while robot.step(timestep) != -1:
    # get current lidar measurements
    scan = lidar.getRangeImage()[0:100]
    
    # minimum distance and its angle
    dis = min(scan)
    min_theta = np.argmin(np.array(scan))

    # move mode: move to closest point
    if move_to_closest:
    
        # self roration until robot face to closest point
        if min_theta == 90:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            
            # move until distance<threshold
            if dis > threshold - lidar_to_rob[0]:
                leftMotor.setVelocity(1)
                rightMotor.setVelocity(1)
            else:
                # stop move, switch to follow_wall mode
                move_to_closest = False
                follow_wall = True
        else: 
            leftMotor.setVelocity(-1)
            rightMotor.setVelocity(1)
            
    # follow_wall mode   
    if follow_wall:
        vl = (0.5 - 0.1625) * np.pi / 12.8
        vr = (0.5 + 0.1625) * np.pi / 12.8
        # if robot parallel to the wall
        if min_theta == 0:
            if scan[0] > 0.5:
                leftMotor.setVelocity(2*vl/0.195)
                rightMotor.setVelocity(2*vr/0.195)
            else:               
                leftMotor.setVelocity(2*vr/0.195)
                rightMotor.setVelocity(2*vl/0.195) 
                
        # if small bias, rotate to corecction
        elif min_theta>0 and min_theta<2:
            leftMotor.setVelocity(2*vr/0.195)
            rightMotor.setVelocity(2*vl/0.195)
            
        # if large differenz, self rotation
        elif min_theta > 2:
            leftMotor.setVelocity(1)
            rightMotor.setVelocity(-1)
                
    pass




