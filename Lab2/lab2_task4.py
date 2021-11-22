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

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

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

def turn90(turn):
    starting = math.degrees(imu.getRollPitchYaw()[2] + math.pi)
    if(turn == 'r'):
        while robot.step(timestep) != -1 and ((math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > 0) or (math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > -90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 0)):
            motor[1].setVelocity(-2)
            motor[0].setVelocity(2)
    else:
        while robot.step(timestep) != -1 and ((math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > 0) or (math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > -90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 0)):
            motor[1].setVelocity(2)
            motor[0].setVelocity(-2)
    motor[1].setVelocity(0)
    motor[0].setVelocity(0) 

def findCylinder(x):
    while robot.step(timestep) != -1 and (camera.getRecognitionObjects() == []):
        leftMotor.setVelocity(x * 1.5)
        rightMotor.setVelocity(x * -1.5)
    motor[0].setVelocity(0)
    motor[1].setVelocity(0)

def adjust(x):
    if (len(camera.getRecognitionObjects()) == 0):
        return
    elif ( len(camera.getRecognitionObjects()[0].get_position_on_image()) == 0):
        return
    position = camera.getRecognitionObjects()[0].get_position()[0]
    while robot.step(timestep) != -1 and (camera.getRecognitionObjects()[0].get_position()[0] > 0.05 or camera.getRecognitionObjects()[0].get_position()[0] < -0.05):
        obj = camera.getRecognitionObjects()[0]
        leftMotor.setVelocity(x * 1.5)
        rightMotor.setVelocity(x * -1.5)

    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)


def moveAlongWall(wall):
    velocity = 4
    if (wall == 'l'):
        err = dist[0].getValue() - inchesToMeters(target_distance)
        if (err < 0):
            motor[0].setVelocity(velocity / 0.8)
            motor[1].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
        else:
            motor[0].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
            motor[1].setVelocity((velocity) / 0.8)
    elif(wall == 'r'):
        err = dist[1].getValue() - inchesToMeters(target_distance)
        if (err < 0):
            motor[0].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
            motor[1].setVelocity(velocity / 0.8)
        else:
            motor[0].setVelocity((velocity) / 0.8)
            motor[1].setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)

def followWall(wall):
    if(metersToInches(dist[0].getValue()) > target_distance and metersToInches(dist[1].getValue()) > target_distance and metersToInches(dist[2].getValue()) > target_distance):
        starting_pos = leftposition_sensor.getValue() * 0.8
        while(robot.step(timestep) != -1 and leftposition_sensor.getValue()*0.8 - starting_pos < 5):
            motor[0].setVelocity(5)
            motor[1].setVelocity(5)

        turn90('r' if turns == 'l' else 'l')
    else:
        moveAlongWall(wall)
        if(metersToInches(dist[2].getValue()) <= target_distance):
            turn90(turns)

def isCentered():
    if(camera.getRecognitionObjects()[0].get_position()[0] < 0.05 and camera.getRecognitionObjects()[0].get_position()[0] > -0.05):
        return True
    else:
        return False

proportional_gain = 1
target_distance = 8
time = 180
start_time = robot.getTime()
turns = 'r'

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and (robot.getTime() - start_time < time):
    if(len(camera.getRecognitionObjects()) != 0 and not isCentered() and metersToInches(dist[2].getValue()) > 10 and metersToInches(dist[0].getValue()) > 10 and metersToInches(dist[1].getValue()) > 10):
        adjust(1 if turns == 'l' else -1)
    elif (len(camera.getRecognitionObjects()) != 0 and isCentered()):
        motor[0].setVelocity(5)
        motor[1].setVelocity(5)

        if(metersToInches(dist[2].getValue()) < target_distance):
            if (camera.getRecognitionObjects()[0].get_size_on_image()[0] * camera.getRecognitionObjects()[0].get_size_on_image()[1] > 2700):
                motor[0].setVelocity(0)
                motor[1].setVelocity(0)
                exit()
            else:
                turn90(turns)
    else:
        if(dist[0].getValue() < dist[1].getValue()):
            followWall('l')
        else:
            followWall('r')

        
        
 


# Enter here exit cleanup code.
motor[0].setVelocity(0)
motor[1].setVelocity(0)