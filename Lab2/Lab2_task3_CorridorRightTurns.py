"""lab2_task3corridor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, DistanceSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


dist = [leftDistanceSensor, rightDistanceSensor, frontDistanceSensor]
motor = [leftMotor, rightMotor]

def metersToInches(m):
    return m * 39.3701

def inchesToMeters(inch):
    return inch * 0.0254

def turn90():
    starting = math.degrees(imu.getRollPitchYaw()[2] + math.pi)
    while robot.step(timestep) != -1 and ((math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > 0) or (math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > -90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 0)):
        motor[1].setVelocity(-math.pi)
        motor[0].setVelocity(math.pi)
    motor[1].setVelocity(0)
    motor[0].setVelocity(0)

def saturationFunc(arg):
    if (arg > 6.28 or arg == 0):
        return 6.28
    elif (arg < -6.28):
        return -6.28
    else: 
        return arg 

def followWall(wall):
    velocity = 4
    if (wall == 0):
        print("correct")
        err = dist[0].getValue() - inchesToMeters(target_distance)
        if (err < 0):
            motor[0].setVelocity(velocity / 0.8)
            motor[1].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
        else:
            motor[0].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
            motor[1].setVelocity((velocity) / 0.8)
    elif(wall == 1):
        print("incorrect")
        err = dist[1].getValue() - inchesToMeters(target_distance)
        if (err < 0):
            motor[0].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
            motor[1].setVelocity(velocity / 0.8)
        else:
            motor[0].setVelocity((velocity) / 0.8)
            motor[1].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)

proportional_gain = 1
target_distance = 7
time = 180
# wall = chooseWall()
start_time = robot.getTime()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and (robot.getTime() - start_time < time):
    if(metersToInches(dist[2].getValue()) <= 7):
        turn90()
    elif(dist[0].getValue() < dist[1].getValue()):
        followWall(0)
    else:
        followWall(1)
    

# Enter here exit cleanup code.
motor[0].setVelocity(0)
motor[1].setVelocity(0)