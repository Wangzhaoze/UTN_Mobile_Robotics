"""diff_drive_solution_two_steps controller."""

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

# set wheel velocities (in rad)
# a velocity of 2*pi means that the wheel will make one turn per second
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

### two steps solution consists on moving on two half-circles,
### with a diameter of half of the distance between start and goal

### the radius of the linear velocity of the right and left wheel
### are computed as v_r=omega*(R+l/2)
### and v_l = omega*(R-l/2), with R = 1m here in the exercise
### dividing the two equations, we get v_r/v_l = (R+l/2)/(R-l/2) =: rl_frac
### rl_frac will be used to find the wheel speed

l = 0.325
R = 1.0
rl_frac = (R+l/2)/(R-l/2)

# we will not use the max speed in order to make more precise motions
wheel_dim = 0.195
base_speed_rad = MAX_SPEED/32.0 # in rad/sec
base_speed_lin = base_speed_rad*(wheel_dim/2.0) # the speed of the wheel on a plane, m/s

# the base_speed will be used for the wheel on the outer circle
# to compute the time the robot needs to drive a halh-circle,
# we will use the path travelled by the outer wheel

dist_outer = (R+l/2)*np.pi
# timesteps required (note that timestep is in ms)
# we introduce here some imprecision through timestep discretization
ts_for_half_circle = int((dist_outer/base_speed_lin)/(timestep/1000.0))

ts_count = 0
while robot.step(timestep) != -1:
    # for the first half-circle, the right wheel is outside
    # the speed of the left wheel is determined by dividing by rl_frac
    if ts_count < ts_for_half_circle:
        leftMotor.setVelocity(base_speed_rad/rl_frac)
        rightMotor.setVelocity(base_speed_rad)
    # for the second half-circle, we swap the velocities
    elif ts_count < 2*ts_for_half_circle:
        leftMotor.setVelocity(base_speed_rad)
        rightMotor.setVelocity(base_speed_rad/rl_frac)
    else:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        
    ts_count +=1
    pass

# Enter here exit cleanup code.
