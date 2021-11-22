"""lab1_task3 controller."""

from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


H = 10
W = 20

a = 100
b = 80

X = 5
Y = 5


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

# function to convert from degrees to radians
def degreesToRadians(deg):
    return (deg * math.pi / 180) 
    
# function to convert from radians to degrees
def radiansToDegrees(rad):
    return ((rad + math.pi) * 180) / math.pi
    
wheelRad = 0.8
wheelCircum = 2 * wheelRad * math.pi
wheelDist = 2.28
dmid = wheelDist/2

def getPosition():
    return [leftposition_sensor.getValue() * wheelRad, rightposition_sensor.getValue() * wheelRad]

def yaw():
    return radiansToDegrees(imu.getRollPitchYaw()[2])
    
def normalize(y):
    if(y >= 359): 
        return 0
    else: 
        return y
    
def turn(degrees):
    startingAngle = normalize(yaw())
  
  #  rad = degreesToRadians(degrees) 
  #  distanceLeft = rad * -dmid
  #  distanceRight = rad * dmid
    
  #  velocityLeft = distanceLeft / Y
  #  velocityRight = distanceRight / Y
    
  #  phiLeft = velocityLeft / wheelRad
  #  phiRight = velocityRight / wheelRad
    
    while robot.step(timestep) != -1:
        leftMotor.setVelocity(-math.pi)
        rightMotor.setVelocity(math.pi)
        
        if (yaw() - startingAngle >= degrees):
            break
    
def move(distance):
    velocity = X / (wheelRad)
    if (velocity > 6.28):
        print("velocity exceeds 6.26")
        exit()
    positionStart = getPosition()[0]
    time_start = robot.getTime()
    while robot.step(timestep) != -1:
        leftMotor.setVelocity(velocity)
        rightMotor.setVelocity(velocity)
           
        if (getPosition()[0] - positionStart >= distance):
            break   

turn(90)      # used to orient the robot counter-clockwise
move((W / 2))
turn(180 - a)
move(H / math.cos(a))
turn((180 - b))
move(W)
turn(180 - b)
move(H / math.cos(b))
turn(180 - a) 
move((W / 2))

# Enter here exit cleanup code.
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)