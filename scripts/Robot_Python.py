#!/usr/bin/python
import Unitree_Python_sdk
import time
unitree_robot = Unitree_Python_sdk.Unitree_Robot()

motion_time = 0
while True:
    time.sleep(0.002)
    # Uncomment the following line to control the pose of robot
    # The four arguments are: roll, pitch, yaw, bodyHeight.
     
    # state = unitree_robot.robot_pose(0.0, 0.0, 1, 0.0)
    
    # Uncomment the following line to control the movement of robot
    # The arguments are: gait type, forward speed, sideway speed, rotate speed, speed level and body height.
    # Gait type: 0.idle  1.trot  2.trot running  3.climb stair
    # Forward speed: unit: m/s -1.0 ~ 1.0
    # Sideway speed: unit: m/s -1.0 ~ 1.0
    # Rotate speed: unit: rad/s -1.0 ~ 1.0
    # Speed level: 0. default low speed. 1. medium speed 
    # Body height: unit: m, default: 0.28m
    
    #state = unitree_robot.robot_walking(gaitType = 1, forwardSpeed = -0.1, sidewaySpeed = 0.0, rotateSpeed = 0.0, speedLevel = 0, bodyHeight = 0.0)

    motion_time += 1
    if (motion_time < 1000):
        state = unitree_robot.robot_dance(1)
    if (motion_time >= 1000 and motion_time < 2000):
        state = unitree_robot.quit_dance()
    if (motion_time >= 2000):
        state = unitree_robot.robot_walking(gaitType = 1, forwardSpeed = 0.1, sidewaySpeed = 0.0, rotateSpeed = 0.0, speedLevel = 0, bodyHeight = 0.0)

    
        
    
    


    # return state of robot    
    # imu                       //rpy[0], rpy[1], rpy[3]
    # gaitType                  // 0.idle  1.trot  2.trot running  3.climb stair
    # footRaiseHeight           // (unit: m, default: 0.08m), foot up height while walking
    # position                  // (unit: m), from own odometry in inertial frame, usually drift
    # bodyHeight                // (unit: m, default: 0.28m),
    # velocity                  // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
    # yawSpeed                  // (unit: rad/s), rotateSpeed in body frame        
    # footPosition2Body         // foot position relative to body
    # footSpeed2Body            // foot speed relative to body
    # footForce
    
    # print(state.imu.rpy[0])
