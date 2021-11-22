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
# currentCell = 15
# currentCell = 16

numParticles = 80

importance_factor = 1/80

#initial board of particles
particleBoard = [[[80/16, importance_factor] for i in range(4)] for i in range(4)]

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

def rotateToStartingAngle(x):
    while robot.step(timestep) != -1 and (not (getAngle() < x + 1 and getAngle() > x - 1)):
        leftMotor.setVelocity(-2)
        rightMotor.setVelocity(2)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
def moveForward():
    starting_pos = leftposition_sensor.getValue() * 0.8
    while(robot.step(timestep) != -1 and leftposition_sensor.getValue()*0.8 - starting_pos < 10):
        leftMotor.setVelocity(4)
        rightMotor.setVelocity(4)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
def direction():
    if (getAngle() >= 0 and getAngle() < 5) or (getAngle() > 355 and getAngle() < 360):
        return 's'
    if (getAngle() > 85 and getAngle() < 95):
        return 'e'
    elif(getAngle() > 175 and getAngle() < 185):
        return 'n'
    elif(getAngle() > 265 and getAngle() < 275):
        return 'w'


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

def findCell(c):
    for i in range(4):
        for j in range(4):
            if (board[i][j][0] == c):
                return i, j

def distributeParticles(cells):
    global particleBoard, importance_factor, numParticles
    
    num_particles_per_board = numParticles/len(cells)
    particleBoard = [[[0, importance_factor] for i in range(4)] for i in range(4)]            
    
    for cell in cells:
        i, j = findCell(cell)
        particleBoard[i][j][0] = num_particles_per_board
        
    
def getWall(s):
    if (mToIn(frontDistanceSensor.getValue()) <= 5):
        s += 'W'
    else:
        s += 'O'
    return s
    
def getWallConfig():
    starting_angle = getAngle()
    while robot.step(timestep) != -1 and direction() != 'w':
        leftMotor.setVelocity(-2)
        rightMotor.setVelocity(2)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    wall_config = ""
    wall_config = getWall(wall_config)
    turn90('r')
    wall_config = getWall(wall_config)
    turn90('r')
    wall_config = getWall(wall_config)
    turn90('r')
    wall_config = getWall(wall_config)
    turn90('r')
    
    rotateToStartingAngle(starting_angle)
    return wall_config
    
def getCell():
    # global currentCell
    c = 0
    
    wall_config = getWallConfig()
    
    cells = []
    for i in board:
        for j in i:
            if (j[2] == wall_config):
                cells.append(j[0])
                
    if len(cells) == 1:
        c = cells[0]
    else:
        for r in range(len(cells)):
            distributeParticles(cells)
            moveForward()
            wall = getWallConfig()
            dir = direction()
            for cell in cells:
                i, j = findCell(cell)
                if (dir == 'e'):
                    if (not (board[i][j][2] == wall_config and board[i][j + r + 1][2] == wall)):
                        cells.remove(cell)
                elif(dir == 'w'):
                    if (not (board[i][j][2] == wall_config and board[i][j - r - 1][2] == wall)):
                        cells.remove(cell)
                elif(dir == 'n'):
                    if (not (board[i][j][2] == wall_config and board[i - r - 1][j][2] == wall)):
                        cells.remove(cell)
                else:
                    if (not (board[i][j][2] == wall_config and board[i + r + 1][j][2] == wall)):
                        cells.remove(cell)

                
                if (len(cells) == 1):
                    markVisited(cells[0])
                    if (r > 0):
                        for i in range(1, r):
                            if (dir == 'e'):
                                k = (i) * 1
                            elif(dir == 'w'):
                                k = (i) * -1
                            elif(dir == 'n'):
                                k = (i) * -4
                            else:
                                k = (i) * 4                            
                            markVisited(cells[0] + k)
                            
                    if (dir == 'e'):
                        k = (r + 1) * 1
                    elif(dir == 'w'):
                        k = (r + 1) * -1
                    elif(dir == 'n'):
                        k = (r + 1) * -4
                    else:
                        k = (r + 1) * 4
                    c = cells[0] + k
                    break
    return c
        
def getRobotCoordinates(c):
    coordinates = [[[-15, 15], [-5, 15], [5, 15], [15, 15]], 
                    [[-15, 5], [-5, 5], [5, 5], [15, 5]], 
                    [[-15, -5], [-5, -5], [5, -5], [15, -5]], 
                    [[-15, -15], [-5, -15], [5, -15], [15, -15]]]
    for i in range(4):
        for j in range(4):
            if (board[i][j][0] == c):
                return coordinates[i][j][0], coordinates[i][i][1]
           
def printPose(c):
    # cell = getCell() 
    x, y = getRobotCoordinates(c)
    orientation = round(imu.getRollPitchYaw()[2], 5)
    # getCell()
    print('(' + str(x) + ', ' + str(y) + ', ' + str(c) + ', ' + str(orientation) + ')')    

def isVisited(cell):
    for i in range(4):
        for j in range(4):
            if(board[i][j][0] == cell):
                 if (board[i][j][1] == 'X'):
                     return True
                 else:
                     return False
    
def markVisited(c):
    for i in range(4):
        for j in range(4):
            if (board[i][j][0] == c):
                board[i][j][1] = 'X'
    printPose(c)
    printBoard()
    print('-----------------------------------------------------------')
   
def moveToNextCell():
    global currentCell
    
    moveForward()
        
    if(direction() == 'e'):
        currentCell += 1
    elif(direction() == 'w'):
        currentCell -= 1
    elif(direction() == 'n'):
        currentCell -= 4
    else:
        currentCell += 4
    
    getWallConfig()
    
starting_position = leftposition_sensor.getValue()

start_time = robot.getTime()

currentCell = getCell()

while robot.step(timestep) != -1 and robot.getTime() - start_time < 180:
    markVisited(currentCell)
    if(complete()):
        break
    # moveToNextCell()
    if (currentCell == 16 and direction() == 's'):
        turn90('r')
    if (currentCell == 1 or currentCell == 4 or currentCell == 13 or currentCell == 16):
        if (currentCell == 1):
            if (direction() == 'w'):
                turn90('l')
                turn90('l')
        elif(currentCell == 4):
            turn90('l' if not direction() == 'e' else 'r')
        elif(currentCell == 16):
            turn90('r' if direction() == 's' else 'l')
        else:
            turn90('r' if direction() == 's' else 'l')
     
        # turn90('l')
        
        starting_position = leftposition_sensor.getValue()
        # continue
    if (currentCell == 5):
        turn90('r')
    elif(currentCell == 6):
        turn90('r')
    elif(currentCell == 10):
        turn90('l')
    elif(currentCell == 11):
        turn90('l')
    elif(currentCell == 3 and direction() == 'n'):
        turn90('l')
         
    moveToNextCell()

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)