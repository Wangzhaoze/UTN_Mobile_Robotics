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



# set wheel velocities (in rad)
# a velocity of 2*pi means that the wheel will make one turn per second


leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
#deg = 0.325 * np.pi / 0.195
#leftMotor.setPosition(deg)
#rightMotor.setPosition(-deg)
leftMotor.setVelocity(1)
rightMotor.setVelocity(-1)
# get and enable lidar
lidar = robot.getDevice('Sick LMS 291')
lidar.enable(60)


move = False
threshold = 0.5
lidar_to_rob = [0.08, 0, 0.21]
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # get current lidar measurements
    scan = lidar.getRangeImage()
    dis = min(scan)
    print(dis)

    # rotate until robot face to closest point
    if np.argmin(np.array(scan)) == 90:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        move = True
  
    # move to closest point only if:
    # 1.robot face to closest point
    # 2.dis < threshold - lidar_to_rob[0]
    if move and dis > threshold - lidar_to_rob[0]:
        leftMotor.setVelocity(1)
        rightMotor.setVelocity(1)
    pass




