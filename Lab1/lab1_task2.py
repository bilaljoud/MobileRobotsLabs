"""lab1_task2 controller."""

# FIGURE OUT WHATS WRONG WITH ANGLE AND WRITE READINGS TO FILE FOR PLOT AND DO ERROR MESSAGE

from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# X degrees (0 - 360) [Modify]
X = 20
# Y seconds [Modify]
Y = 1 

degreeDiff = 360 - X

# getting the motors and setting position and velocity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# getting the position sensors
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)
robot.step(timestep)

# function to convert from degrees to radians
def degreesToRadians(deg):
    return (deg * math.pi / 180) 
    
# function to convert from radians to degrees
def radiansToDegrees(rad):
    return ((rad + math.pi) * 180) / math.pi
    
distBetweenWheels = 2.28 
dmid = distBetweenWheels / 2
wheelRad = 0.8
wheelCircum = 2 * wheelRad * math.pi

Xrad = degreesToRadians(X)

distanceLeft = Xrad * -dmid
distanceRight = Xrad * dmid

velocityLeft = distanceLeft / Y
velocityRight = distanceRight / Y

phiLeft = velocityLeft / wheelRad
phiRight = velocityRight / wheelRad

angularVelocity = Xrad / Y

print(phiLeft)
print(phiRight)

if (phiRight > 6.28):
    print ("Error: Velocity exceeds 6.28")
    exit()

time_start = robot.getTime()

file = open("lab1_task2_measurements.txt", "w")
file.write("Angle:\tTime:\n")

leftMotor.setVelocity(phiLeft)
rightMotor.setVelocity(phiRight)

# main loop
while robot.step(timestep) != -1 and (robot.getTime() - time_start < Y):

    leftMotor.setVelocity(phiLeft)
    rightMotor.setVelocity(phiRight)

    print("time: " + str(robot.getTime() - time_start))
    print("degrees: " + str(radiansToDegrees(imu.getRollPitchYaw()[2])))
    file.write(str(radiansToDegrees(imu.getRollPitchYaw()[2])) + "\t" + str(robot.getTime() - time_start)+ "\n")

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
