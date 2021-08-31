/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <cmath>
#include <stdint.h>
#include <stdio.h>

#include "unitree_legged_sdk/pbPlots.hpp"
#include "unitree_legged_sdk/supportLib.hpp"

// using namespace std;
// using namespace UNITREE_LEGGED_SDK;

// class Custom {
// public:
//   Custom() : control(LeggedType::A1, LOWLEVEL), udp() {
//     control.InitCmdData(cmd);
//   }
//   void UDPRecv();
//   void UDPSend();
//   void RobotControl();

//   Control control;
//   UDP udp;
//   LowCmd cmd = {0};
//   LowState state = {0};
//   float qInit[3] = {0};
//   float qDes[3] = {0};
//   float sin_mid_q[3] = {0.0, 1.2, -2.5};
//   float Kp[3] = {0};
//   float Kd[3] = {0};
//   double time_consume = 0;
//   int rate_count = 0;
//   int sin_count = 0;
//   int motiontime = 0;
//   float dt = 0.002; // 0.001~0.01

//   double t_ = 0.5;
//   const float pi = 3.1415;

//   float motor_destination = 0;
//   float read_initial_position = 0;

//   float speed = 3.0;
//   const int target_leg = FL_1;
//   float target_position = -30 * pi / 180;
//   float state_angle = 0;
//   float state_tau = 0;
// };

// void Custom::UDPRecv() { udp.Recv(); }

// void Custom::UDPSend() { udp.Send(); }

// double jointLinearInterpolation(double initPos, double targetPos, double rate) {
//   double p;
//   rate = std::min(std::max(rate, 0.0), 1.0);
//   p = initPos * (1 - rate) + targetPos * rate;
//   return p;
// }

// void Custom::RobotControl() {
//   motiontime++;
//   udp.GetRecv(state);
//   // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);

//   // gravity compensation
//   cmd.motorCmd[FR_0].tau = -0.65f;
//   cmd.motorCmd[FL_0].tau = +0.65f;
//   cmd.motorCmd[RR_0].tau = -0.65f;
//   cmd.motorCmd[RL_0].tau = +0.65f;

//   // if( motiontime >= 100){
//   if (motiontime >= 0) {
//     // first, get record initial position
//     // if( motiontime >= 100 && motiontime < 500){
//     if (motiontime >= 0 && motiontime < 10) {
//       // qInit[0] = state.motorState[FR_0].q;
//       // qInit[1] = state.motorState[FR_1].q;
//       // qInit[2] = state.motorState[FR_2].q;
//       read_initial_position = state.motorState[target_leg].q;
//     }
//     // second, move to the origin point of a sine movement with Kp Kd
//     // if( motiontime >= 500 && motiontime < 1500){
//     if (motiontime >= 10 && motiontime < 400) {
//       rate_count++;
//       double rate = rate_count / 200.0; // needs count to 200
//       Kp[0] = 5.0;
//       Kp[1] = 5.0;
//       Kp[2] = 5.0;
//       Kd[0] = 1.0;
//       Kd[1] = 1.0;
//       Kd[2] = 1.0;

//       // qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
//       // qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
//       // qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
//       // motor_destination = jointLinearInterpolation(read_initial_position,
//       //                                              target_position, rate);
//     }
//     double sin_joint1, sin_joint2;
//     // last, do sine wave
//     if (0) {
//       sin_count++;
//       sin_joint1 = 0.6 * sin(3 * M_PI * sin_count / 1000.0);
//       sin_joint2 = -0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
//       // qDes[0] = sin_mid_q[0];
//       // qDes[1] = sin_mid_q[1];
//       qDes[1] = target_position + sin_joint2;
//       // qDes[2] = sin_mid_q[2];
//     }

//     // cmd.motorCmd[FR_0].q = qDes[0];
//     // cmd.motorCmd[FR_0].dq = 0;
//     // cmd.motorCmd[FR_0].Kp = Kp[0];
//     // cmd.motorCmd[FR_0].Kd = Kd[0];
//     // cmd.motorCmd[FR_0].tau = -0.65f;

//     // cmd.motorCmd[FR_1].q = qDes[1];
//     // cmd.motorCmd[FR_1].dq = 0;
//     // cmd.motorCmd[FR_1].Kp = Kp[1];
//     // cmd.motorCmd[FR_1].Kd = Kd[1];
//     // cmd.motorCmd[FR_1].tau = 0.0f;
//     std::vector<double> t = createDomain(0.0, 1.0, 0.001);
//     for(int i = 0; i < t.size(); i++)
//     {
//       if (t[i] < t_)
//       {
//             Eigen::Vector3d FeetPosition;
//             FeetPosition = CalculateBezierStance(t[i] / t_, 1.0, 0.0);
//             x.push_back(FeetPosition[0]);
//             y.push_back(FeetPosition[1]);
//             z.push_back(FeetPosition[2]);
//       }
//       else
//       {
//             Eigen::Vector3d FeetPosition;
//             FeetPosition = CalculateBezierSwing((t[i] - t_) / (1.0 - t_), 1.0, 0.0);
//             x.push_back(FeetPosition[0]);
//             y.push_back(FeetPosition[1]);
//             z.push_back(FeetPosition[2]);
//       }

//     }
    
//     cmd.motorCmd[target_leg].q = motor_destination;
//     cmd.motorCmd[target_leg].dq = speed;
//     cmd.motorCmd[target_leg].Kp = Kp[2];
//     cmd.motorCmd[target_leg].Kd = Kd[2];
//     cmd.motorCmd[target_leg].tau = 0.8f;

//     cmd.motorCmd[target_leg].q = motor_destination;
//     cmd.motorCmd[target_leg].dq = speed;
//     cmd.motorCmd[target_leg].Kp = Kp[2];
//     cmd.motorCmd[target_leg].Kd = Kd[2];
//     cmd.motorCmd[target_leg].tau = 0.8f;

//     cmd.motorCmd[target_leg].q = motor_destination;
//     cmd.motorCmd[target_leg].dq = speed;
//     cmd.motorCmd[target_leg].Kp = Kp[2];
//     cmd.motorCmd[target_leg].Kd = Kd[2];
//     cmd.motorCmd[target_leg].tau = 0.8f;

//     // while(1)
//     {
//       // state_angle = state.motorState[FL_1].q;
//       // state_tau = state.motorState[FL_1].tauEst;

//       // cout << " current angle : " << state_angle * 180 / pi << endl;
//       // cout << " current tau : " << state_tau << endl;
//       // usleep(1000);
//     }
//   }

//   if (motiontime > 10) {
//     // control.PositionLimit(cmd);
//     // control.PowerProtect(cmd, state, 1);
//     // control.PositionProtect(cmd, state, 0.087);
//   }

//   udp.SetSend(cmd);
// }

// int main(void) {
//   std::cout << "Control level is set to LOW-level." << std::endl
//             << "WARNING: Make sure the robot is hung up." << std::endl
//             << "Press Enter to continue..." << std::endl;
//   std::cin.ignore();

//   Custom custom;

//   LoopFunc loop_control("control_loop", custom.dt,
//                         boost::bind(&Custom::RobotControl, &custom));
//   LoopFunc loop_udpSend("udp_send", custom.dt, 3,
//                         boost::bind(&Custom::UDPSend, &custom));
//   LoopFunc loop_udpRecv("udp_recv", custom.dt, 3,
//                         boost::bind(&Custom::UDPRecv, &custom));

//   loop_udpSend.start();
//   loop_udpRecv.start();
//   loop_control.start();

//   while (1) {
//     sleep(10);
//   };

//   return 0;
// }


int main(int argc, char **argv)
{
  const double StanceTime = 0.5; // 0 ~ 1
  const double V = 1.0;
  const double angle = 0.0;
  std::vector<double> CurrentTime = createDomain(0.0, 1.0, 0.01);
  Eigen::Vector3d rotation(0, 0, 0);
  Eigen::Vector3d center(0, 0, 0);
  
  std::vector<double> x, y, z;

  RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();


  for(auto currtime : CurrentTime)
  {
    Eigen::Vector3d FeetPosition;
    Eigen::VectorXd MotorRadian(12);
    Eigen::VectorXd MotorDeg(12);
    
    if (currtime < StanceTime)
    {
      double StanceRatio = currtime / StanceTime;
      FeetPosition = CalculateBezierStance(StanceRatio, V, angle);
      x.push_back(FeetPosition[0]);
      y.push_back(FeetPosition[1]);
      z.push_back(FeetPosition[2]);
      std::cout << " Stance !!   " << " stanceratio : " << StanceRatio << "    ";
      std::cout << " X : " << FeetPosition[0] * pow(10, 3) 
                << " Y : " << FeetPosition[1] * pow(10, 3) 
                << " Z : " << FeetPosition[2] * pow(10, 3) << std::endl;
    }
    else
    {
      double SwingRatio = (currtime - StanceTime) / (1.0 - StanceTime);
      FeetPosition = CalculateBezierSwing(SwingRatio, V, angle);
      x.push_back(FeetPosition[0]);
      y.push_back(FeetPosition[1]);
      z.push_back(FeetPosition[2]);
      std::cout << std::endl;
      std::cout << " Swing !!   " << " swingratio : " << SwingRatio << "    ";
      std::cout << " X : " << FeetPosition[0] * pow(10, 3) 
                << " Y : " << FeetPosition[1] * pow(10, 3) 
                << " Z : " << FeetPosition[2] * pow(10, 3) << std::endl;
    }

    Eigen::Matrix4d RobotLegPosition;
    RobotLegPosition << FeetPosition[0] * pow(10, 3), FeetPosition[1] * pow(10, 3), FeetPosition[2] * pow(10, 3), 1,
                        100, -100, -100, 1,
                        -100, -100, 1, 1,
                        -100, -100, -100, 1;
    // RobotLegPosition << 100, -100, 100, 1,
    //                     100, -100, -100, 1,
    //                     -100, -100, 100, 1,
    //                     -100, -100, -100, 1;
    
    MotorRadian = CalcIK(RobotLegPosition, rotation, center);
    
    for(int i = 0; i < MotorRadian.size(); i++)
    {
      MotorDeg[i] = Rad2deg(MotorRadian[i]);
    }
    
    std::cout << MotorDeg.transpose() << std::endl;  
    
    std::vector<double> MotorRad;
    MotorRad = EigenXdTovec(MotorRadian);

    for(auto i : MotorRad)
    {
      // cmd.motorCmd[target_leg].q = motor_destination;
      // cmd.motorCmd[target_leg].dq = speed;
      // cmd.motorCmd[target_leg].Kp = Kp[2];
      // cmd.motorCmd[target_leg].Kd = Kd[2];
      // cmd.motorCmd[target_leg].tau = 0.8f;
    }
  }
    DrawScatterPlot(imageReference, 1000, 300, &x, &z);
    std::vector<double> *pngData = ConvertToPNG(imageReference -> image);
    WriteToFile(pngData, "bezier_curve.png");
    DeleteImage(imageReference -> image);
 


  return 0;
}
