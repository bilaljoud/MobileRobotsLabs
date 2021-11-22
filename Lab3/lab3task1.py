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

# visited = [['.' for i in range(4)] for i in range(4)]
board = [[[1, '.'], [2, '.'], [3, '.'], [4, '.']], 
        [[5, '.'], [6, '.'], [7, '.'], [8, '.']], 
        [[9, '.'], [10, '.'], [11, '.'], [12, '.']], 
        [[13, '.'], [14, '.'], [15, '.'], [16, '.']]]
       
# figure out how to do list comprehension version of above matrix 
# board =

currentCell = 13
currentX = -15
currentY = -15

def meToIn(m):
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
            rightMotor.setVelocity(1)
            leftMotor.setVelocity(-1)
    rightMotor.setVelocity(0)
    leftMotor.setVelocity(0) 

# def pidSide():
    # if (mToIn(leftDistanceSensor.getValue()) < 10 or mToIn(rightDistanceSensor.getValue()) < 10):
        # wall = 0 if mToIn(leftDistanceSensor.getValue()) < 10 else 1
    
        # target_distance = 5
        # velocity = 4
        # if (wall == 0):
            # err = leftDistanceSensor.getValue() - inchesToMeters(target_distance)
            # if (err < 0):
                # leftMotor.setVelocity(velocity / 0.8)
                # rightMotor.setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
            # else:
                # leftMotor.setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
                # rightMotor.setVelocity((velocity) / 0.8)
        # elif(wall == 1):
            # err = rightDistanceSensor.getValue() - inchesToMeters(target_distance)
            # if (err < 0):
                # leftMotor.setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)
                # rightMotor.setVelocity(velocity / 0.8)
            # else:
                # leftMotor.setVelocity((velocity) / 0.8)
                # rightMotor.setVelocity((velocity - (abs(err) * proportional_gain)) / 0.8)

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
    
    # for i in visited:
        # for j in i:
            # print(j, end = ' ')
        # print()

# def determineCell():
   ### should I just return the current cell?
    # pass

def complete():
    for i in range(1, 16):
       if(not isVisited(i)):
           return False
    return True 
           
def printPose():
    cell = currentCell
    x = currentX
    y = currentY
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
        currentX += 10
    elif(direction() == 'w'):
        currentCell -= 1
        currentX -= 10
    elif(direction() == 'n'):
        currentCell -= 4
        currentY += 10
    else:
        currentCell += 4
        currentY -= 10    
    
    
starting_position = leftposition_sensor.getValue()

markVisited()
# Main loop:
# - perform simulation steps until Webots is stopping the controller
start_time = robot.getTime()
while robot.step(timestep) != -1 and robot.getTime() - start_time < 180:
    if(complete()):
        break
    moveToNextCell()
    markVisited()
    if (currentCell == 1 or currentCell == 4 or currentCell == 13 or currentCell == 16):
        if (currentCell == 1):
            turn90('l')
        elif(currentCell == 13):
            turn90('l')
        else:
            turn90('l' if direction() == 'n' or direction() == 'e' else 'r')
        starting_position = leftposition_sensor.getValue()
        continue
        

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)