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



using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
  Custom(uint8_t level) : safe(LeggedType::A1), udp(level) {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  float curr_qDes[3] = {0, 0.52, -0.785};
  float final_qDes[3] = {0};
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
  const double pi = M_PI;
  int pre_rate_count = 0;
  int bezier_rate_count = 0;

  const double StanceTime = 0.5; // 0 ~ 1
  const double V = 2.0;
  const double angle = 0.0;
  // std::vector<double> CurrentTime = createDomain(0.0, 1.0, 0.0001);
  Eigen::Vector3d rotation = {0, 0, 0};
  Eigen::Vector3d center = {0, 0, 0};
  std::vector<double> x, y, z;
  
  bool PreparePose = false;
  bool BezierCurve = false;
  bool StanceMode = false; // false : SwingMode

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
        BezierCurve = true;
    }

    // move to the prepare pose for bezier curve
    if(PreparePose)
    {
      pre_rate_count++;
      double rate = pre_rate_count / 200.0;

      curr_qDes[0] = jointLinearInterpolation(qInit[0], final_qDes[0], rate);
      curr_qDes[1] = jointLinearInterpolation(qInit[1], final_qDes[1], rate);
      curr_qDes[2] = jointLinearInterpolation(qInit[2], final_qDes[2], rate);      
    
      ControlMotor(cmd, FR_0, curr_qDes[0], 0.0f);
      ControlMotor(cmd, FR_1, curr_qDes[1], 0.0f);
      ControlMotor(cmd, FR_2, curr_qDes[2], 0.0f);
      usleep(10000);
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
      
    if(BezierCurve)
    {  
      // RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();
        bezier_rate_count++;
        double rate = bezier_rate_count / 200.0;
        double currtime = jointLinearInterpolation(0.0, 1.0, rate);
        if(rate == 1 ) bezier_rate_count = 0;

        Eigen::Vector4d BezierPositon_UpShoulder;
        Eigen::VectorXd MotorRadian(12);
        Eigen::VectorXd MotorDeg(12);
        

        if (currtime < StanceTime)
        {
          StanceMode = true; // StanceMode
          double StanceRatio = currtime / StanceTime;
          BezierPositon_UpShoulder = CalculateBezierStance(StanceRatio, V, angle);

          std::cout << " Stance !!   " << " stanceratio : " << StanceRatio << "    ";
          std::cout << " X : " << BezierPositon_UpShoulder[0]
                    << " Y : " << BezierPositon_UpShoulder[1] 
                    << " Z : " << BezierPositon_UpShoulder[2] << std::endl;
        }
        else
        {
          StanceMode = false; // SwingMode
          double SwingRatio = (currtime - StanceTime) / (1.0 - StanceTime);
          BezierPositon_UpShoulder = CalculateBezierSwing(SwingRatio, V, angle);

          std::cout << std::endl;
          std::cout << " Swing !!   " << " swingratio : " << SwingRatio << "    ";
          std::cout << " X : " << BezierPositon_UpShoulder[0] 
                    << " Y : " << BezierPositon_UpShoulder[1] 
                    << " Z : " << BezierPositon_UpShoulder[2] << std::endl;
        }

        
        
        Eigen::Matrix4d RobotLegPosition;
        RobotLegPosition << (TransDownShoulder2RobotCenter1 * BezierPositon_UpShoulder),
                            (TransDownShoulder2RobotCenter2 * BezierPositon_UpShoulder),                           
                            (TransDownShoulder2RobotCenter3 * BezierPositon_UpShoulder),                            
                            (TransDownShoulder2RobotCenter4 * BezierPositon_UpShoulder);
        
        MotorRadian = CalcIK(center, rotation, RobotLegPosition);
        std::cout << RobotLegPosition << std:: endl;

        std::cout <<  Rad2deg(MotorRadian[0]) << "    " <<
                      Rad2deg(MotorRadian[1]) << "    " <<
                      Rad2deg(MotorRadian[2]) << "    " <<std::endl;
                      // Rad2deg(MotorRadian[9]) << "    " <<
                      // Rad2deg(MotorRadian[10]) << "    " <<
                      // Rad2deg(MotorRadian[11]) << std::endl;
        

        if(StanceMode)
        {
          ControlMotor(cmd, FL_0, 0.0);
          ControlMotor(cmd, FL_1, (float)(-MotorRadian[1]), 3.0f);
          ControlMotor(cmd, FL_2, (float)(-MotorRadian[2]), 3.0f);

          // ControlMotor(cmd, FL_0, 0.0);
          // ControlMotor(cmd, FL_1, (float)(-MotorRadian[4]), 3.0f);
          // ControlMotor(cmd, FL_2, (float)(-MotorRadian[5]), 3.0f);
  
          // ControlMotor(cmd, RR_0, 0.0);
          // ControlMotor(cmd, RR_1, (float)(-MotorRadian[10]), 3.0f);
          // ControlMotor(cmd, RR_2, (float)(-MotorRadian[11]), 3.0f);          
                 
        }
        else  // Swing
        {
          ControlMotor(cmd, FL_0, 0.0);
          ControlMotor(cmd, FL_1, (float)(-MotorRadian[1]));
          ControlMotor(cmd, FL_2, (float)(-MotorRadian[2]));

          // ControlMotor(cmd, FL_0, 0.0);
          // ControlMotor(cmd, FL_1, (float)(-MotorRadian[4]));
          // ControlMotor(cmd, FL_2, (float)(-MotorRadian[5]));
          
          // ControlMotor(cmd, RR_0, 0.0);
          // ControlMotor(cmd, RR_1, (float)(-MotorRadian[10]));
          // ControlMotor(cmd, RR_2, (float)(-MotorRadian[11]));
             
        }

        


      

        // DrawScatterPlot(imageReference, 1000, 300, &x, &z);
        // std::vector<double> *pngData = ConvertToPNG(imageReference -> image);
        // WriteToFile(pngData, "bezier_curve.png");
        // DeleteImage(imageReference -> image);

    }

    if (motiontime > 10) 
    {
      // safe.PositionLimit(cmd);
      // safe.PowerProtect(cmd, state, 1);
      // safe.PositionProtect(cmd, state, 0.087);
    }

    udp.SetSend(cmd);
    usleep(10000);
  }

}

int main(void) 
{
  std::cout << "Control level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);

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

    










