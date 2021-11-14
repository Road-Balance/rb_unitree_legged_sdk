/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <array>
#include <math.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <msgpack.hpp>

using namespace UNITREE_LEGGED_SDK;

class RobotInterface
{
public:
    RobotInterface(): safe(LeggedType::A1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    HighState getState();
    void RobotControl(uint8_t mode, uint8_t gaitType, uint8_t speedLevel,
                        float footRaiseHeight, float bodyHeight, 
                        float roll, float pitch, float yaw,
                        float forwardSpeed, float sidewaySpeed, float rotateSpeed);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    float dt = 0.002;     // 0.001~0.01
};


void RobotInterface::UDPRecv()
{
    udp.Recv();
}

void RobotInterface::UDPSend()
{  
    udp.Send();
}

HighState RobotInterface::getState()
{
    udp.GetRecv(state);
    //printf("%f\n", state.velocity[0]);
    return state;
}

void RobotInterface::RobotControl(uint8_t mode, uint8_t gaitType, uint8_t speedLevel,
                                  float footRaiseHeight, float bodyHeight, 
                                  float roll, float pitch, float yaw,
                                  float forwardSpeed, float sidewaySpeed, float rotateSpeed) 
{
    //udp.GetRecv(state);

    //printf("%f\n", state.imu.rpy[1]);

    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;

    if (mode == 1){
        cmd.mode = mode;
        cmd.euler[0] = roll;
        cmd.euler[1] = pitch;
        cmd.euler[2] = yaw;
        cmd.bodyHeight = bodyHeight;
    }
    else if (mode == 2) {
        cmd.mode = mode;
        cmd.gaitType = gaitType;
        cmd.speedLevel = speedLevel;
        cmd.footRaiseHeight = footRaiseHeight;
        cmd.bodyHeight = bodyHeight;
        cmd.velocity[0] = forwardSpeed;
        cmd.velocity[1] = sidewaySpeed;
        cmd.yawSpeed = rotateSpeed;
    }
    else{
        cmd.mode = mode;
    }


    udp.SetSend(cmd);
}

namespace py=pybind11;

PYBIND11_MODULE(robot_interface_high_level, m) 
{
    py::class_<RobotInterface>(m, "RobotInterface")
    .def(py::init())
    .def("UDPRecv", &RobotInterface::UDPRecv)
    .def("UDPSend", &RobotInterface::UDPSend)
    .def("getState", &RobotInterface::getState)
    .def("robotControl", &RobotInterface::RobotControl);

    py::class_<Cartesian>(m, "Cartesian")
        .def(py::init<>())
        .def_readwrite("x", &Cartesian::x)
        .def_readwrite("y", &Cartesian::y)
        .def_readwrite("z", &Cartesian::z);

    py::class_<IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("quaternion", &IMU::quaternion)
        .def_readwrite("gyroscope", &IMU::gyroscope)
        .def_readwrite("accelerometer", &IMU::accelerometer)
        .def_readwrite("rpy", &IMU::rpy)
        .def_readwrite("temperature", &IMU::temperature);

    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighState::levelFlag)
        .def_readwrite("commVersion", &HighState::commVersion)
        .def_readwrite("robotID", &HighState::robotID)
        .def_readwrite("SN", &HighState::SN)
        .def_readwrite("bandWidth", &HighState::bandWidth)
        .def_readwrite("mode", &HighState::mode)
        .def_readwrite("progress", &HighState::progress)
        .def_readwrite("imu", &HighState::imu)
        .def_readwrite("gaitType", &HighState::gaitType)
        .def_readwrite("footRaiseHeight", &HighState::footRaiseHeight)
        .def_readwrite("position", &HighState::position)
        .def_readwrite("bodyHeight", &HighState::bodyHeight)
        .def_readwrite("velocity", &HighState::velocity)
        .def_readwrite("yawSpeed", &HighState::yawSpeed)
        .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
        .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
        .def_readwrite("footForce", &HighState::footForce)
        .def_readwrite("footForceEst", &HighState::footForceEst)
        .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
        .def_readwrite("reserve", &HighState::reserve)
        .def_readwrite("crc", &HighState::crc);
}
