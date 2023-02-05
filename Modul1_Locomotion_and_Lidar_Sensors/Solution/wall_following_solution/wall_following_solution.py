"""wall_following_solution controller."""

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

### There are many wall following algorithms, 
### this one is a simple solution that will work in many cases, but is not universal
### here the idea is to keep the wall left at a distance in a certain ragne
### we begin by moving the robot to an initial pose where the wall is actually left
### after that, there are four cases we need to consider:
### 1. the robot is not too close and not too far from the left wall, in which case it moves forward
### 2. the robot is too far from the left wall, in that case it turns to the left
### 3. the robot is too close to the left wall, in that case the robot turns right
### 4. the robot faces a wall in front, in that case the robot also turns right

# set wheel velocities (in rad)
# a velocity of 2*pi means that the wheel will make one turn per second
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# keep the code from closest point part

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
at_closest_point = False # we want to move to the closest wall point first
wall_following_initialized = False
while robot.step(timestep) != -1:
    # get current lidar measurements
    scan = lidar.getRangeImage()
    curr_dist = scan[90] # use central beam to detect front wall
    front_dist = curr_dist   
    left_dist = scan[0] # use left-most beam to measure distance to the left
    left_too_close = left_dist<0.3
    left_too_far = left_dist > 0.5 or left_dist == 0.0 # we handle invalid measurements as too far
    wall_in_front = front_dist < 0.3
    if not at_closest_point:
    ############## closest point part ###########################
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
            at_closest_point = True
    ################ wall following ############################################
    else:
        # we need to make sure that the wall is left of the robot
        if not wall_following_initialized:
            leftMotor.setVelocity(base_speed_rad)
            rightMotor.setVelocity(-base_speed_rad)
            wall_following_initialized = not left_too_far
        # wall following after initialization
        else:
            # too far from the wall, turn left on a circle
            if left_too_far and not wall_in_front:
                leftMotor.setVelocity(0.1*base_speed_rad)
                rightMotor.setVelocity(base_speed_rad)
            else:
                # no wall, not too close -> move forward
                if not wall_in_front and not left_too_close:                  
                    leftMotor.setVelocity(base_speed_rad)
                    rightMotor.setVelocity(base_speed_rad)
                # too close to the left wall, or wall in front, turn right in place
                else:
                    leftMotor.setVelocity(base_speed_rad)
                    rightMotor.setVelocity(-base_speed_rad)
    ts_count += 1
    pass

# Enter here exit cleanup code.