#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <cmath>
#include <stdint.h>
#include <stdio.h>
    
double jointLinearInterpolation(double initPos, double targetPos, double rate) 
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}  

int main()
{
    Eigen::Vector3d rotation = {0, 0, 0};
    Eigen::Vector3d center = {0, 0, 0};
    Eigen::Matrix4d RobotLegPosition;
    Eigen::VectorXd MotorRadian(12);
    int bezier_rate_count = 0;
    const double StanceTime = 0.5; // 0 ~ 1
    const double V = 2.0;
    const double angle = 0.0;
    bool BezierCurve = true;
    bool StanceMode = false; // false : SwingMode
    
  
    
    // double L(0.3610), W(0.094);
    // double l1(0.0838), l2(0.2), l3(0.2);
    // Eigen::Ventor4d BezierPositon_UpShoulder;
    // BezierPositon_UpShoulder <<  
    // double k = W/2 + l1;

    // RobotLegPosition <<     k,      -k,     k,      -k,
    //                         -l2-l3, -l2-l3, -l2-l3, -l2-l3,
    //                         L/2,    L/2,    -L/2,   -L/2,
    //                         1,      1,      1,      1;
    
    while(BezierCurve)
    {  
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
    
    }
    Eigen::Vector4d a;
    a << W/2 + l1, 0, -L/2, 1;
    std::cout << TransDownShoulder2RobotCenter2 * a << std::endl;  

    MotorRadian = CalcIK(center, rotation, RobotLegPosition);

    std::cout << MotorRadian << std::endl;

    return 0;
}