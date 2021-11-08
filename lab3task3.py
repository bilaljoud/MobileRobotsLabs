"""lab3task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#enable distance sensors
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

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

board = [[[1, '.', "WWOW"], [2, '.', "OWOW"], [3, '.', "OWOO"], [4, '.', "OWWO"]], 
        [[5, '.', "WWOO"], [6, '.', "OWWO"], [7, '.', "WOWO"], [8, '.', "WOWO"]], 
        [[9, '.', "WOWO"], [10, '.', "WOOW"], [11, '.', "OOWW"], [12, '.', "WOWO"]], 
        [[13, '.', "WOOW"], [14, '.', "OWOW"], [15, '.', "OWOW"], [16, '.', "OOWW"]]]
       
# change this var to change starting cell, then move robot to specified cell
currentCell = 15

def mToIn(m):
    return m * 39.3701

def inToM(inch):
    return inch * 0.0254
    
def getAngle():
    return math.degrees(imu.getRollPitchYaw()[2]) + 180

def turn90(turn):
    starting = math.degrees(imu.getRollPitchYaw()[2] + math.pi)
    if(turn == 'r'):
        while robot.step(timestep) != -1 and ((math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > 0) or (math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > -90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 0)):
            rightMotor.setVelocity(-2)
            leftMotor.setVelocity(2)
    else:
        while robot.step(timestep) != -1 and ((math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > 0) or (math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting > -90 and math.degrees(imu.getRollPitchYaw()[2] + math.pi) - starting < 0)):
            rightMotor.setVelocity(2)
            leftMotor.setVelocity(-2)
    rightMotor.setVelocity(0)
    leftMotor.setVelocity(0) 


def direction():
    if(getAngle() >= 0 and getAngle() < 5):
        return 'w'
    if (getAngle() > 85 and getAngle() < 95):
        return 's'
    elif(getAngle() > 175 and getAngle() < 185):
        return 'e'
    elif(getAngle() > 265 and getAngle() < 275):
        return 'n'


def printBoard():
    for i in range(4):
        for j in range(4):
            print(board[i][j][1], end = ' ')
        print()

def complete():
    for i in range(1, 16):
       if(not isVisited(i)):
           return False
    return True 
           
def printPose():
    cell = currentCell 
    x, y = getRobotCoordinates()
    orientation = round(imu.getRollPitchYaw()[2], 5)
    print('(' + str(x) + ', ' + str(y) + ', ' + str(cell) + ', ' + str(orientation) + ')')
    
def isVisited(cell):
    for i in range(4):
        for j in range(4):
            if(board[i][j][0] == cell):
                 if (board[i][j][1] == 'X'):
                     return True
                 else:
                     return False
    
def markVisited():
    for i in range(4):
        for j in range(4):
            if (board[i][j][0] == currentCell):
                board[i][j][1] = 'X'
    printPose()
    printBoard()
    print('-----------------------------------------------------------')
   
def moveToNextCell():
    global currentCell, currentX, currentY
    
    
    if (direction() == 'w' and isVisited(currentCell - 1)):
        turn90('l')
    elif(direction() == 'e' and isVisited(currentCell + 1)):
        turn90('l')
    elif(direction() == 'n' and isVisited(currentCell - 4)):
        turn90('l')
    elif(direction() == 's' and isVisited(currentCell + 4)):
        turn90('l')
    
    starting_pos = leftposition_sensor.getValue() * 0.8
    while(robot.step(timestep) != -1 and leftposition_sensor.getValue()*0.8 - starting_pos < 10):
        leftMotor.setVelocity(4)
        rightMotor.setVelocity(4)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
        
    if(direction() == 'e'):
        currentCell += 1
    elif(direction() == 'w'):
        currentCell -= 1
    elif(direction() == 'n'):
        currentCell -= 4
    else:
        currentCell += 4
    
    
starting_position = leftposition_sensor.getValue()

start_time = robot.getTime()

while robot.step(timestep) != -1 and robot.getTime() - start_time < 180:
    # markVisited()
    # if(complete()):
        # break
    moveToNextCell()
    # if (currentCell == 1 or currentCell == 4 or currentCell == 13 or currentCell == 16):
        if (currentCell == 1):
            turn90('l')
        elif(currentCell == 13):
            turn90('l')
        el:
            turn90('l' if direction() == 'n' or direction() == 'e' else 'r')
        # turn90('l')
        
        # starting_position = leftposition_sensor.getValue()
        continue
    # moveToNextCell()

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)