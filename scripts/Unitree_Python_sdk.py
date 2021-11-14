#!/usr/bin/python
import sys
sys.path.append('/path/to/unitree_legged_sdk-3.3.2/build') # Edit the path to "build" folder on your computer

import robot_interface_high_level as robot_interface

class Unitree_Robot():

    unitree_robot = robot_interface.RobotInterface()
    robot_state = robot_interface.HighState()
    
    
    def __init__(self):
        self.mode = 0
        self.gaitType = 0
        self.speedLevel = 0
        self.footRaiseHeight = 0.0
        self.forwardSpeed = 0.0
        self.sidewaySpeed = 0.0
        self.rotateSpeed = 0.0
        self.bodyHeight = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.quit_dance_time = 0

    def cmd_init(self):
        self.mode = 0
        self.gaitType = 0
        self.speedLevel = 0
        self.footRaiseHeight = 0.0
        self.forwardSpeed = 0.0
        self.sidewaySpeed = 0.0
        self.rotateSpeed = 0.0
        self.bodyHeight = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.quit_dance_time = 0
    
    def send_UDP(self):
        self.unitree_robot.UDPSend()

    def recv_UDP(self):
        self.unitree_robot.UDPRecv()
        
    def robot_control(self):
        self.unitree_robot.robotControl(self.mode, self.gaitType, self.speedLevel,
                                    self.footRaiseHeight, self.bodyHeight, 
                                    self.roll, self.pitch, self.yaw,
                                    self.forwardSpeed, self.sidewaySpeed, self.rotateSpeed)
    
    
    def robot_walking(self, gaitType = 1, forwardSpeed = 0.0, sidewaySpeed = 0.0, 
                      rotateSpeed = 0.0, speedLevel = 0, bodyHeight = 0.0, footRaiseHeight = 0.0):
        self.cmd_init()
        self.gaitType = gaitType
        self.speedLevel = speedLevel
        self.footRaiseHeight = footRaiseHeight
        self.forwardSpeed = forwardSpeed
        self.sidewaySpeed = sidewaySpeed
        self.rotateSpeed = rotateSpeed
        self.bodyHeight = bodyHeight
        self.mode = 2
        robot_state = self.unitree_robot.getState()
        self.recv_UDP()
        self.robot_control()
        self.send_UDP()
        return robot_state
    

    def robot_pose(self, roll, pitch, yaw, bodyHeight):
        self.cmd_init()
        self.bodyHeight = bodyHeight
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.mode = 1
        robot_state = self.unitree_robot.getState()
        self.recv_UDP()     
        self.robot_control()
        self.send_UDP()
        return robot_state 
    
    def jump_yaw(self):
        self.cmd_init()
        self.mode = 10
        robot_state = self.unitree_robot.getState()
        self.recv_UDP()
        self.robot_control()
        self.send_UDP()
        return robot_state       

    def straight_hand(self):
        self.cmd_init()
        self.mode = 11
        robot_state = self.unitree_robot.getState()
        self.recv_UDP()
        self.robot_control()
        self.send_UDP()
        return robot_state      
    
    def robot_dance(self, dance_genre):
        self.cmd_init()
        if (dance_genre == 1):
            self.mode = 12
        elif (dance_genre == 2):
            self.mode = 13
        robot_state = self.unitree_robot.getState()
        self.recv_UDP()
        self.robot_control()
        self.send_UDP()
        return robot_state

    def quit_dance(self):
        self.quit_dance_time += 1
        if (self.quit_dance_time < 500):
            self.mode = 2
        if (self.quit_dance_time >= 500):
            self.mode = 1
        robot_state = self.unitree_robot.getState()
        self.recv_UDP()
        self.robot_control()
        self.send_UDP()
        return robot_state        
        
        
