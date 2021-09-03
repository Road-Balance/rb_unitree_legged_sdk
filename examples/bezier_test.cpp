/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <cmath>
#include <stdint.h>
#include <stdio.h>
// #include <conio.h>

#include "unitree_legged_sdk/pbPlots.hpp"
#include "unitree_legged_sdk/supportLib.hpp"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
  Custom() : control(LeggedType::A1, LOWLEVEL), udp() {
    control.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Control control;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  float curr_qDes[3] = {0};
  float final_qDes[3] = {0};
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
  const double pi = M_PI;
  int pre_rate_count = 0;
  int bezier_rate_count = 0;

  const double StanceTime = 0.5; // 0 ~ 1
  const double V = 2.0;
  const double angle = 0.0;
  std::vector<double> CurrentTime = createDomain(0.0, 1.0, 0.0001);
  Eigen::Vector3d rotation = {0, 0, 0};
  Eigen::Vector3d center = {0, 0, 0};
  std::vector<double> x, y, z;
  
  bool PreparePose = false;
  bool BezierCurve = false;

};


 




void Custom::UDPRecv() { udp.Recv(); }

void Custom::UDPSend() { udp.Send(); }

double jointLinearInterpolation(double initPos, double targetPos, double rate) 
{
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void Custom::RobotControl() 
{
  motiontime++;
  udp.GetRecv(state);
  // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);

  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  if (motiontime >= 0) 
  {
    // read initial position
    if (motiontime >= 0 && motiontime < 10) 
    {
      qInit[0] = state.motorState[FR_0].q;
      qInit[1] = state.motorState[FR_1].q;
      qInit[2] = state.motorState[FR_2].q;
      
      if(motiontime == 9)
        PreparePose = true;
    }

    // move to the prepare pose for bezier curve
    if(false)
    {
      pre_rate_count++;
      double rate = pre_rate_count / 200.0;

      curr_qDes[0] = jointLinearInterpolation(qInit[0], final_qDes[0], rate);
      curr_qDes[1] = jointLinearInterpolation(qInit[1], final_qDes[1], rate);
      curr_qDes[2] = jointLinearInterpolation(qInit[2], final_qDes[2], rate);      
    
      ControlMotor(FR_0, curr_qDes[0], 0.8f);
      ControlMotor(FR_1, curr_qDes[1], 0.8f);
      ControlMotor(FR_2, curr_qDes[2], 0.8f);
      
      if( curr_qDes == final_qDes)
      {
        BezierCurve = true;
        PreparePose = false;
        // while(1)
        // {
        //   int key = getch();
        //   if(key == 13)
        //     break;
        // }
      }
    }
      
    if(true)
    {  
      // RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();
        bezier_rate_count++;
        double rate = bezier_rate_count / 200.0;
        double currtime = jointLinearInterpolation(0.0, 1.0, rate);
        if(rate == 1 ) bezier_rate_count = 0;

        Eigen::Vector3d FeetPosition;
        Eigen::VectorXd MotorRadian(12);
        Eigen::VectorXd MotorDeg(12);
        
        if (currtime < StanceTime)
        {
          double StanceRatio = currtime / StanceTime;
          FeetPosition = CalculateBezierStance(StanceRatio, V, angle);
          // x.push_back(FeetPosition[0]);
          // y.push_back(FeetPosition[1]);
          // z.push_back(FeetPosition[2]);
          std::cout << " Stance !!   " << " stanceratio : " << StanceRatio << "    ";
          std::cout << " X : " << FeetPosition[0]
                    << " Y : " << FeetPosition[1] 
                    << " Z : " << FeetPosition[2] << std::endl;
        }
        else
        {
          double SwingRatio = (currtime - StanceTime) / (1.0 - StanceTime);
          FeetPosition = CalculateBezierSwing(SwingRatio, V, angle);
          // x.push_back(FeetPosition[0]);
          // y.push_back(FeetPosition[1]);
          // z.push_back(FeetPosition[2]);
          std::cout << std::endl;
          std::cout << " Swing !!   " << " swingratio : " << SwingRatio << "    ";
          std::cout << " X : " << FeetPosition[0] 
                    << " Y : " << FeetPosition[1] 
                    << " Z : " << FeetPosition[2] << std::endl;
        }

        Eigen::Matrix4d RobotLegPosition;
        RobotLegPosition << FeetPosition[0],  FeetPosition[1], -0.32 + FeetPosition[2], 1,
                            0.1, -0.1, -0.1, 1,
                            -0.1, -0.1, 0.1, 1,
                            -0.1, -0.1, -0.1, 1;
        // RobotLegPosition << 100, -100, 100, 1,
        //                     100, -100, -100, 1,
        //                     -100, -100, 100, 1,
        //                     -100, -100, -100, 1;
        
        // MotorRadian = CalcIK(RobotLegPosition, rotation, center);
        
        Eigen::Vector4d LegPoint;
        LegPoint << RobotLegPosition(0, 0), RobotLegPosition(0, 2), RobotLegPosition(0, 1), 1.0;

        Eigen::Vector3d theta = LegIK(LegPoint);
        std::cout <<  Rad2deg(theta[0]) << "    " <<
                      Rad2deg(theta[1]) << "    " <<
                      Rad2deg(theta[2]) << std::endl;
        


        // for(int i = 0; i < MotorRadian.size(); i++)
        // {
        //   MotorDeg[i] = Rad2deg(MotorRadian[i]);
        // }
        
        // std::cout << MotorDeg[0] << "        " <<
        // MotorDeg[1] << "        " <<
        // MotorDeg[2] << std::endl;  
        
        // std::vector<double> MotorRad;
        // MotorRad = EigenXdTovec(MotorRadian);

        
        // ControlMotor(FR_0, MotorRadian[3], 0.8f);
        // ControlMotor(FR_1, MotorRadian[4], 0.8f);
        // ControlMotor(FR_2, (float)(-MotorRadian[2]), 0.8f);
        cmd.motorCmd[FR_2].q = (float)(-theta[2]);
        cmd.motorCmd[FR_2].dq = 0.0;
        cmd.motorCmd[FR_2].Kp = 5.0;
        cmd.motorCmd[FR_2].Kd = 1.0;
        cmd.motorCmd[FR_2].tau = 0.8f;

        cmd.motorCmd[FR_1].q = (float)(-theta[1]);
        cmd.motorCmd[FR_1].dq = 0.0;
        cmd.motorCmd[FR_1].Kp = 5.0;
        cmd.motorCmd[FR_1].Kd = 1.0;
        cmd.motorCmd[FR_1].tau = 0.8f;
      

        // DrawScatterPlot(imageReference, 1000, 300, &x, &z);
        // std::vector<double> *pngData = ConvertToPNG(imageReference -> image);
        // WriteToFile(pngData, "bezier_curve.png");
        // DeleteImage(imageReference -> image);

    }

    if (motiontime > 10) 
    {
      // control.PositionLimit(cmd);
      // control.PowerProtect(cmd, state, 1);
      // control.PositionProtect(cmd, state, 0.087);
    }

    udp.SetSend(cmd);
  }

}

int main(void) 
{
  std::cout << "Control level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom;

  LoopFunc loop_control("control_loop", custom.dt,
                        boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3,
                        boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3,
                        boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1) 
  {
    sleep(10);
  };

  return 0;
}

    










