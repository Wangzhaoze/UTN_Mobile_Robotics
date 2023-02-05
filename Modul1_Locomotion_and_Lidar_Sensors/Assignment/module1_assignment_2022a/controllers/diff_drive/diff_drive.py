"""diff_drive controller."""

from controller import Robot
import numpy as np
# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


i=0
while robot.step(timestep) != -1:
    # I hope robot reach the goal point in 12.8s = 400 * 32ms
    # compute velocity of wheels
    vl = (2 - 0.1625) * np.pi / 12.8
    vr = (2 + 0.1625) * np.pi / 12.8
    
    # set robot angle velocity
    leftMotor.setVelocity(2*vl/0.195)
    rightMotor.setVelocity(2*vr/0.195)
    
    # time in 400 * 32ms
    if i > 400:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        
    i += 1
    pass


