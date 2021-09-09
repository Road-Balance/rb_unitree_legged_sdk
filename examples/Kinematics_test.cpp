#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <cmath>
#include <stdint.h>
#include <stdio.h>

int main()
{
    Eigen::Vector3d rotation = {0, 0, 0};
    Eigen::Vector3d center = {0, 0, 0};
    Eigen::Matrix4d RobotLegPosition;
    Eigen::VectorXd MotorRadian(12);
    // double L(0.3610), W(0.094);
    // double l1(0.0838), l2(0.2), l3(0.2);

    double k = W/2 + l1;

    RobotLegPosition <<     k,      -k,     k,      -k,
                            -l2-l3, -l2-l3, -l2-l3, -l2-l3,
                            L/2,    L/2,    -L/2,   -L/2,
                            1,      1,      1,      1;
                        

    MotorRadian = CalcIK(center, rotation, RobotLegPosition);

    std::cout << MotorRadian << std::endl;

    return 0;
}