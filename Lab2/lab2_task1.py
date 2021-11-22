"""lab2_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#initialization of motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#distance sensors 
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)



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
        
def pidFront():
    time_left = time - (robot.getTime() - startTime)
    current_distance = frontDistanceSensor.getValue()
    error = current_distance - target_distance
    u_t = error * proportional_gain
    u_r_t = saturationFunc(u_t / 0.8)
    leftMotor.setVelocity(u_r_t)
    rightMotor.setVelocity(u_r_t)
    
def adjustPath():
    current_distance = metersToInches(leftDistanceSensor.getValue())
    current_velocity = leftMotor.getVelocity()
    print(current_velocity)
    if (current_distance <= 2.5):
        error = 2.5 - current_distance
        u_t = inchesToMeters(error) * proportional_gain
        u_r_t = saturationFunc(u_t)
        print("u_r(t): " + str(u_r_t))
        rightMotor.setVelocity(saturationFunc(current_velocity + u_r_t))
        leftMotor.setVelocity(current_velocity)
    elif (current_distance >= 5.5):
        error = 5.5 - current_distance
        u_t = inchesToMeters(error) * proportional_gain
        u_r_t = saturationFunc(u_t)
        print("u_r(t): " + str(u_r_t))
        rightMotor.setVelocity(saturationFunc(current_velocity - (current_velocity*u_r_t)))
        leftMotor.setVelocity(current_velocity)
    else:
        leftMotor.setVelocity(current_velocity)
        rightMotor.setVelocity(current_velocity)
        
        
    current_distance = metersToInches(rightDistanceSensor.getValue())
    current_velocity = leftMotor.getVelocity()
    print(current_velocity)
    if (current_distance <= 2.5):
        error = 2.5 - current_distance
        u_t = inchesToMeters(error) * proportional_gain
        u_r_t = saturationFunc(u_t)
        print("u_r(t): " + str(u_r_t))
        rightMotor.setVelocity(saturationFunc(current_velocity + (current_velocity*u_r_t)))
        leftMotor.setVelocity(current_velocity)
    elif (current_distance >= 5.5):
        error = 5.5 - current_distance
        u_t = inchesToMeters(error) * proportional_gain
        u_r_t = saturationFunc(u_t)
        print("u_r(t): " + str(u_r_t))
        rightMotor.setVelocity(saturationFunc(current_velocity - (current_velocity*u_r_t)))
        leftMotor.setVelocity(current_velocity)
    else:
        leftMotor.setVelocity(current_velocity)
        rightMotor.setVelocity(current_velocity)
    
    
# max time to complete the motion, program stops after this many seconds       
time = 30
proportional_gain = 5

target_distance = inchesToMeters(10) 
r2_t = inchesToMeters(2.5)       

startTime = robot.getTime()
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and (robot.getTime() - startTime < time):
    pidFront()  
    adjustPath()
    
    print ("Front: " + str(metersToInches(frontDistanceSensor.getValue())))
    print ("Left: " + str(metersToInches(leftDistanceSensor.getValue())))
    print ("Right: " + str(metersToInches(rightDistanceSensor.getValue())))


leftMotor.setVelocity(0)
rightMotor.setVelocity(0)