"""lab1_task4 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# radius 1 and 2
R1 = 5
R2 = 5

# speed and time
X = 6
Y = 15

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#getting the position sensors
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)
robot.step(timestep)

# function to convert from radians to degrees
def radiansToDegrees(rad):
    return ((rad + math.pi) * 180) / math.pi
    
# distance is circumference since travel full circles
distance1 = R1 * 2 * math.pi
distance2 = R2 * 2 * math.pi 

time1 = distance1 / X
time2 = distance2 / X

time = time1 + time2
if (time > Y):
    print ("Error: Cannot complete motion in specified time")
    quit()


dmid = 1.14 # since distance btw/ wheels is 2.28

def circle1():
  #  r1Time = (R1 / (R1 + R2)) * Y
    angularVelocity = X / R1
    lwheelVel = angularVelocity * (R1 - dmid)
    rwheelVel = angularVelocity * (R1 + dmid)
    
    if (rwheelVel > 6.28):
        print("Error: wheel velocity exceeds 6.28")
        exit()
    
    startingYaw = radiansToDegrees(imu.getRollPitchYaw()[2])
    
    while robot.step(timestep) != -1:
        leftMotor.setVelocity(lwheelVel)
        rightMotor.setVelocity(rwheelVel)
        
        if (radiansToDegrees(imu.getRollPitchYaw()[2]) <= 0.5):
            break

def circle2():
 #   r2Time = (R2 / (R1 + R2)) * Y
    angularVelocity = X / R2
    lwheelVel = angularVelocity * (R2 + dmid)
    rwheelVel =  angularVelocity * (R2 - dmid)
    
    if (lwheelVel > 6.28):
        print("Error: wheel velocity exceeds 6.28")
        exit()
    
    startingYaw = radiansToDegrees(imu.getRollPitchYaw()[2])
    
    while robot.step(timestep) != -1:
        leftMotor.setVelocity(lwheelVel)
        rightMotor.setVelocity(rwheelVel)
        
        if (radiansToDegrees(imu.getRollPitchYaw()[2]) <= 0.6):
            break

circle1()
circle2()

# Enter here exit cleanup code.
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)