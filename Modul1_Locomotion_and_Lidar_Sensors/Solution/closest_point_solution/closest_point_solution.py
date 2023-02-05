"""closest_point_solution controller."""

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

### we will approach the problem with the following algorithm
### first, the robot will turn in place for 360 degrees to scan the scene
### we will use only the central beam of the scan, here scan[90]
### during scanning, in each step we update the minimal distance
### after the scan, the robot will turn again, until it revisits 
### the direction with the minimal value, and then moves forward

l = 0.325
wheel_dim = 0.195

# we will not use the max speed in order to make more precise motions
base_speed_rad = MAX_SPEED/32.0 # in rad/sec
base_speed_lin = base_speed_rad*(wheel_dim/2.0) # the speed of the wheel on a plane, m/s

# linear distance a wheel will travel when turning
dist_turn = l*np.pi
# timesteps required for 360 degree turn (note that timestep is in ms)
# we introduce here some imprecision through timestep discretization
ts_for_turn = int((dist_turn/base_speed_lin)/(timestep/1000.0))

# get and enable lidar
lidar = robot.getDevice('Sick LMS 291')
lidar.enable(60)

ts_count = 0
min_dist = 1000.0 # pick a large initial value
epsilon = 0.01 # the sensor might be noisy, pick a tolerance value
found_min_dist = False
while robot.step(timestep) != -1:
    # get current lidar measurements
    scan = lidar.getRangeImage()
    curr_dist = scan[90]    
    # scan turn
    if ts_count < ts_for_turn:  
        if curr_dist > 0.0 and curr_dist < min_dist:
            min_dist = curr_dist
        leftMotor.setVelocity(-base_speed_rad)
        rightMotor.setVelocity(base_speed_rad)
    # turn again to find the min_dist again
    elif not found_min_dist and ts_count < 2*ts_for_turn:
        if (curr_dist - epsilon) < min_dist:
            found_min_dist = True
        leftMotor.setVelocity(-base_speed_rad)
        rightMotor.setVelocity(base_speed_rad)
    # move forward until close enough to the wall
    elif found_min_dist and curr_dist > 0.5-0.08: # adjust goal distance by the offset of the lidar
        leftMotor.setVelocity(base_speed_rad)
        rightMotor.setVelocity(base_speed_rad)
    else:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
    ts_count += 1
    pass

# Enter here exit cleanup code.