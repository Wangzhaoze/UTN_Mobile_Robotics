"""diff_drive_solution_three_steps controller."""

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

### three step solution constitutes turning in place towards the goal,
### moving forward to the goal position, and finally turning in place 
### into the goal direction
    
### first, we pre-compute the necessary parameters to perform the motion
### we will need to turn in-place twice by 90 degrees, 
### and move on a line for two meters

l = 0.325
wheel_dim = 0.195

# we will not use the max speed in order to make more precise motions
base_speed_rad = MAX_SPEED/32.0 # in rad/sec
base_speed_lin = base_speed_rad*(wheel_dim/2.0) # the speed of the wheel on a plane, m/s

# linear distance a wheel will travel when turning
dist_turn = l*(np.pi/4.0)
# timesteps required (note that timestep is in ms)
# we introduce here some imprecision through timestep discretization
ts_for_turn = int((dist_turn/base_speed_lin)/(timestep/1000.0))

# similarely, for the line moving part:
dist_line = 4.0 
ts_for_line = int((dist_line/base_speed_lin)/(timestep/1000.0))

ts_count = 0
while robot.step(timestep) != -1:
    # turn 90 degrees left towards the goal
    if ts_count < ts_for_turn:
        leftMotor.setVelocity(-base_speed_rad)
        rightMotor.setVelocity(base_speed_rad)
    # move to goal position
    elif ts_count < ts_for_line + ts_for_turn:
        leftMotor.setVelocity(base_speed_rad)
        rightMotor.setVelocity(base_speed_rad)
    # turn 90 degrees right
    elif ts_count < ts_for_line + 2*ts_for_turn:
        leftMotor.setVelocity(base_speed_rad)
        rightMotor.setVelocity(-base_speed_rad)
    else:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        
    ts_count +=1
    pass

# Enter here exit cleanup code.
