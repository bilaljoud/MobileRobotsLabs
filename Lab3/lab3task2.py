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

board = [[[1, '.'], [2, '.'], [3, '.'], [4, '.']], 
        [[5, '.'], [6, '.'], [7, '.'], [8, '.']], 
        [[9, '.'], [10, '.'], [11, '.'], [12, '.']], 
        [[13, '.'], [14, '.'], [15, '.'], [16, '.']]]
       
# change this var to change starting cell, then move robot to specified cell
currentCell = 13

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

def color():
    return camera.getRecognitionObjects()[0].get_colors()

def rotateToObject(c):
    while robot.step(timestep) != -1:
        rightMotor.setVelocity(2)
        leftMotor.setVelocity(-2)
        if (len(camera.getRecognitionObjects()) != 0 and color() == c):
                while robot.step(timestep) != -1 and (len(camera.getRecognitionObjects()) != 0) and (camera.getRecognitionObjects()[0].get_position()[0] > 0.05 or camera.getRecognitionObjects()[0].get_position()[0] < -0.05):
                    leftMotor.setVelocity(-1.5)
                    rightMotor.setVelocity(1.5)
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                
                return mToIn(frontDistanceSensor.getValue()) + 3.14
                    

def rotateToStartingAngle(x):
    while robot.step(timestep) != -1 and (not (getAngle() < x + 1 and getAngle() > x - 1)):
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

def findCylinder(c):
    radius = rotateToObject(c)
    if (color() == [1.0, 0.0, 0.0]):
        return 20.0, 20.0, radius
    elif(color() == [0.0, 1.0, 0.0]):
        return -20.0, -20.0, radius
    elif(color() == [0.0, 0.0, 1.0]):
        return 20.0, -20.0, radius
    else:
        return -20.0, 20.0, radius

def getRobotCoordinates():
    startingAngle = getAngle()
    x1, y1, r1 = findCylinder([1.0, 0.0, 0.0])
    
    x2, y2, r2 = findCylinder([0.0, 1.0, 0.0])
    
    x3, y3, r3 = findCylinder([0.0, 0.0, 1.0])
    
    rotateToStartingAngle(startingAngle)
    
    A = (-2 * x1) + (2 * x2)
    B = (-2 * y1) + (2 * y2)
    C = (r1 ** 2) - (r2 ** 2) - (x1 ** 2) + (x2 ** 2) - (y1 ** 2) + (y2 ** 2)
    
    D = (-2 * x2) + (2 * x3)
    E = (-2 * y2) + (2 * y3)
    F = (r2 ** 2) - (r3 ** 2) - (x2 ** 2) + (x3 ** 2) - (y2 ** 2) + (y3 ** 2)
    
    x = ((C * E) - (F * B)) / ((E * A) - (B * D))
    y = ((C * D) - (A * F)) / ((B * D) - (A * E))
    return x, y

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
    markVisited()
    if(complete()):
        break
    moveToNextCell()
    if (currentCell == 1 or currentCell == 4 or currentCell == 13 or currentCell == 16):
        # if (currentCell == 1):
            # turn90('l')
        # elif(currentCell == 13):
            # turn90('l')
        # el:
            # turn90('l' if direction() == 'n' or direction() == 'e' else 'r')
        turn90('l')
        
        starting_position = leftposition_sensor.getValue()
        # continue
    # moveToNextCell()

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)