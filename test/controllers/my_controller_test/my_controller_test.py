"""my_controller_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
"""fruit_sorting_ctrl_opencv controller."""
import time
from controller import Supervisor

robot = Supervisor()
speed = 1.0
target_positions = [-2.0, -3.2, 0.0, -2.4, -3.0]
# Getting and declaring the robot motor  0  0.4  0.1
ur_motors = []
ur_motors.append(robot.getDevice('shoulder_pan_joint'))
ur_motors.append(robot.getDevice('shoulder_lift_joint'))
ur_motors.append(robot.getDevice('elbow_joint'))
ur_motors.append(robot.getDevice('wrist_1_joint'))
ur_motors.append(robot.getDevice('wrist_2_joint'))
ur_motors.append(robot.getDevice('wrist_3_joint'))

for motor in ur_motors: 
 motor.setVelocity(speed)
 
hand_motors = []
hand_motors.append(robot.getDevice('ROBOTIQ 2F-85 Gripper::left finger joint'))
hand_motors.append(robot.getDevice('ROBOTIQ 2F-85 Gripper::right finger joint'))
hand_motors[0].setPosition(hand_motors[0].getMinPosition())
hand_motors[1].setPosition(hand_motors[1].getMinPosition())
ur_motors[5].setPosition(-1.0)

def gotoTarget():
    ur_motors[0].setPosition(target_positions[0])
    
    ur_motors[1].setPosition(target_positions[1])
    
    ur_motors[2].setPosition(target_positions[2])
    
    ur_motors[3].setPosition(target_positions[3])
    
    ur_motors[4].setPosition(target_positions[4])
    
    ur_motors[0].setPosition(-1.56)
    
    
gotoTarget()