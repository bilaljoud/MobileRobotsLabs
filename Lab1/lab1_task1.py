"""lab1_task1 controller."""

from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# X distance to travel [Modify this var]
distance = 10 

# Y seconds to complete motion [Modify this var]
time = 5 


# getting motor devices
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

# setting position and velocity
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# getting the position sensors and enabling them
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)
robot.step(timestep)

wheelRad = 0.8
wheelCircum = 2 * wheelRad * math.pi

def getPosition():
    return [leftposition_sensor.getValue() * wheelRad, rightposition_sensor.getValue() * wheelRad]

posStart = getPosition()

velocity = (distance / time) / 0.8
if(velocity > 6.28):
    print("Error: Velocity exceeds 6.28")
    exit()

time_start = robot.getTime()

file = open("lab1_task1_measurements.txt", "w")
file.write("Position:\tTime:\n")

# main loop
while robot.step(timestep) != -1 and (robot.getTime() - time_start < time):
    leftMotor.setVelocity(velocity)
    rightMotor.setVelocity(velocity)

    print("time: " + str(robot.getTime() - time_start))
    print("pos: " + str(getPosition()[0] - posStart[0]))
    file.write(str(getPosition()[0] - posStart[0]) + "\t" + str(robot.getTime() - time_start) + "\n")

if (getPosition()[0] - posStart[0] + 1 < distance):
    print("Error: Could not complete Motion in specified time")
    
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)