"""lab2_task2 controller."""

from controller import Robot

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

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)


def metersToInches(m):
    return m * 39.37008

def inchesToMeters(i):
    return i * 0.0254

def saturationFunc(arg):
    if (arg > 6.28):
        return 6.28
    elif (arg < -6.28):
        return -6.28
    else: 
        return arg
    
def findCylinder():
    while robot.step(timestep) != -1 and (camera.getRecognitionObjects() == []):
        leftMotor.setVelocity(1.5)
        rightMotor.setVelocity(-1.5)

def adjust():
    if (len(camera.getRecognitionObjects()) == 0):
        return
    elif ( len(camera.getRecognitionObjects()[0].get_position_on_image()) == 0):
        return
    position = camera.getRecognitionObjects()[0].get_position()[0]
    while robot.step(timestep) != -1 and (camera.getRecognitionObjects()[0].get_position()[0] > 0.05 or camera.getRecognitionObjects()[0].get_position()[0] < -0.05):
        obj = camera.getRecognitionObjects()[0]
        leftMotor.setVelocity(1.5)
        rightMotor.setVelocity(-1.5)

    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

def pidFront():
    dist_to_obj = frontDistanceSensor.getValue()
    error = dist_to_obj - target_distance
    u_t = error * proportional_gain
    u_r_t = saturationFunc(u_t / 0.8)
    leftMotor.setVelocity(u_r_t)
    rightMotor.setVelocity(u_r_t)
    
target_distance = inchesToMeters(10) 
time = 30
proportional_gain = 5

camera_width = 80
middle_of_camera = camera_width / 2

start_time = robot.getTime()

while robot.step(timestep) != -1 and (robot.getTime() - start_time < time):
    findCylinder()
    adjust()
    pidFront()

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)