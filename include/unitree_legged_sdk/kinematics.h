#pragma once


#include <iostream>
#include <cmath>
#include <Eigen/Dense> 
#include <vector>
#include "comm.h"

// #include "pbPlots.hpp"
// #include "supportLib.hpp"

const double pi = M_PI; 
double L(0.3610), W(0.094);
double l1(0.0838), l2(0.2), l3(0.2);

Eigen::Matrix4d TransRobotCenter2UpShoulder_1 = (Eigen::Matrix4d() <<   1, 0, 0, -W/2,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, -L/2,     
                                                                        0, 0, 0, 1).finished();

Eigen::Matrix4d TransRobotCenter2UpShoulder_2 = (Eigen::Matrix4d() <<   1, 0, 0, W/2,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, -L/2,     
                                                                        0, 0, 0, 1).finished(); 

Eigen::Matrix4d TransRobotCenter2UpShoulder_3  = (Eigen::Matrix4d() <<  1, 0, 0, -W/2,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, L/2,      
                                                                        0, 0, 0, 1).finished();

Eigen::Matrix4d TransRobotCenter2UpShoulder_4 = (Eigen::Matrix4d() <<   1, 0, 0, W/2,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, L/2,      
                                                                        0, 0, 0, 1).finished();

Eigen::Matrix4d TransDownShoulder2RobotCenter1 = (Eigen::Matrix4d() <<   1, 0, 0, W/2 + l1,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, L/2,     
                                                                        0, 0, 0, 1).finished();

Eigen::Matrix4d TransDownShoulder2RobotCenter2 = (Eigen::Matrix4d() <<   1, 0, 0, -W/2 - l1,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, L/2,     
                                                                        0, 0, 0, 1).finished(); 

Eigen::Matrix4d TransDownShoulder2RobotCenter3  = (Eigen::Matrix4d() <<  1, 0, 0, W/2 + l1,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, -L/2,      
                                                                        0, 0, 0, 1).finished();

Eigen::Matrix4d TransDownShoulder2RobotCenter4 = (Eigen::Matrix4d() <<   1, 0, 0, -W/2 - l1,    
                                                                        0, 1, 0, 0,     
                                                                        0, 0, 1, -L/2,      
                                                                        0, 0, 0, 1).finished();

std::vector<double> createDomain(const double start, const double end, const double step)
{
    std::vector<double> domain;
    for(double i=start; i<end; i += step)
        domain.push_back(i);
    
    return domain;
} 

double Deg2rad(double angle)
{
    double rad = angle * pi / 180;

    return rad;
}

double Rad2deg(double rad)
{       
    double angle = rad * 180 / pi;

    return angle;
}

int Factorial(int num)
{
    int result = 1;
    
    if(num < 0)
        return 0;
    else
    {
        for(int i = 1; i < num + 1; i++)
            result *= i;
    }

    return result;
}

std::vector<double> EigenXdTovec(Eigen::VectorXd EigenXdvec)
{
    std::vector<double> vec;
    vec.resize(EigenXdvec.size());
    for(int i = 0; i < EigenXdvec.size(); i++)
    {
        vec[i] = EigenXdvec[i];
    }

    return vec;
}

int BinomialFactor(int n, int k)
{
    return Factorial(n) / (Factorial(k) * Factorial(n -k));
}

double BezierCurve(double t, int k, double point)
{
    int n = 9;
    return point * BinomialFactor(n, k) * pow(t, (double)k) * pow((1 - t), (double)(n - k));
}

Eigen::Vector4d CalculateBezierStance(double phi_st, double V, double angle)
{   
    double c = cos(Deg2rad(angle));
    double s = sin(Deg2rad(angle));

    double A = 0.005;
    double halfStance = 0.05;
    double p_Stance = halfStance * (1 - 2 * phi_st);
    
    // Eigen::Vector3d StancePosition(c * p_Stance * fabs(V),                       // stanceX                  
    //                                - s * p_Stance * fabs(V),                     // stanceY                       
    //                                - A * cos(pi / (2 * halfStance) * p_Stance)); // stanceZ

    Eigen::Vector4d StancePosition(- A * cos(pi / (2 * halfStance) * p_Stance),                       // stanceX                  
                                   - s * p_Stance * fabs(V) -0.28,                     // stanceY                       
                                   c * p_Stance * fabs(V),
                                   1);
    // std::cout <<  p_Stance << std::endl;
    // *stancePtr = c * p_Stance * fabs(V); // stanceX
    // *(stancePtr + 1) = - s * p_Stance * fabs(V); // stanceY
    // *(stancePtr + 2) = - A * cos(pi / (2 * halfStance) * p_Stance); //stanceZ

    return StancePosition;
}

Eigen::Vector4d CalculateBezierSwing(double phi_sw, double V, double angle)
{
    double c = cos(Deg2rad(angle));
    double s = sin(Deg2rad(angle));

    Eigen::VectorXd X(10), Y(10), Z(10);
    // Eigen::VectorXd X_(10), Y_(10), Z_(10);  

    // X <<    0.112, -0.165, -0.198, -0.198, -0.198, 0.0, 0.0, 0.0, 0.198, 0.198, 0.158, 0.112; // (12,1)
    // Y <<    0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05, 0.0, 0.0; 
    // Z <<    -0.310, -0.310, -0.231, -0.231, -0.231, -0.244, -0.244, -0.211, -0.211, -0.211, -0.310, -0.310;
    
    X <<    -0.05, -0.06, -0.07, -0.07, 0.0, 0.0, 0.07, 0.07, 0.06, 0.05; // (10,1)
    Y <<    0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05; 
    Z<<    0.0, 0.0, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06, 0.0, 0.0;

    X  *= (fabs(V) * c);
    Y  *= (fabs(V) * s);
    Z  *= fabs(V);

    Eigen::Vector4d SwingPositon(0, 0, 0, 0);

    for(int i = 0; i < 10; i++ )
    {
        SwingPositon[2] += BezierCurve(phi_sw, i, X[i]); // swingX
        SwingPositon[0] += BezierCurve(phi_sw, i, Y[i]); // swingY
        SwingPositon[1] = SwingPositon[1] + BezierCurve(phi_sw, i, Z[i]) - 0.28; // swingZ
        SwingPositon[3] = 1.0;
    }

    return SwingPositon;
}

// Control Motor position, speed, tauque

void ControlMotor(UNITREE_LEGGED_SDK::LowCmd &cmd,  const int target_leg, 
                    const float q_, 
                    const float tau_ = 0.0f, 
                    const float dq_ = 0.0, 
                    const float Kp_ = 5.0, 
                    const float Kd_ = 1.0)
{
    cmd.motorCmd[target_leg].q = q_;
    cmd.motorCmd[target_leg].dq = dq_;
    cmd.motorCmd[target_leg].Kp = Kp_;
    cmd.motorCmd[target_leg].Kd = Kd_;
    cmd.motorCmd[target_leg].tau = tau_;

    
}

// Kinematics

Eigen::Matrix4d TransWorld2RobotCenter(Eigen::Vector3d Center, Eigen::Vector3d Rotation) // (x, y, z, r, p, y)
{
    Eigen::Matrix4d Rp, Ry, Rr, T, RyprT;
    // double sHp(sin(pi/2)), cHp(cos(pi/2));
    
    Rp <<   1,            0,              0,              0,
            0,            cos(Rotation[1]),    sin(Rotation[1]),   0,
            0,            -sin(Rotation[1]),    cos(Rotation[1]),    0,
            0,            0,              0,              1;
    
    Ry <<   cos(Rotation[2]),  0,              -sin(Rotation[2]),    0,
            0,            1,              0,              0,
            sin(Rotation[2]), 0,              cos(Rotation[2]),    0,
            0,            0,              0,              1;
    
    Rr <<   cos(Rotation[0]),  sin(Rotation[0]),   0,              0,
            -sin(Rotation[0]),  cos(Rotation[0]),    0,              0,
            0,            0,              1,              0,
            0,            0,              0,              1;

    T <<    1,  0,  0,  -Center[0],
            0,  1,  0,  -Center[1],
            0,  0,  1,  -Center[2],
            0,  0,  0,  1;

    RyprT = Ry * Rp * Rr * T;
    
    return RyprT;

}

// std::vector<Eigen::Matrix4d> TransRobotCenter2UpShoulder()
// {
//     double L(0.3610), W(0.094);// 240 80
//     Eigen::Matrix4d Tlf, Trf, Tlb, Trb;
//     std::vector<Eigen::Matrix4d> Trans(4);
    
//     Tlf << 1, 0, 0, -W/2,    0, 1, 0, 0,     0, 0, 1, -L/2,     0, 0, 0, 1;
//     Trf << 1, 0, 0, W/2,    0, 1, 0, 0,     0, 0, 1, -L/2,     0, 0, 0, 1;
//     Tlb << 1, 0, 0, -W/2,    0, 1, 0, 0,     0, 0, 1, L/2,      0, 0, 0, 1;
//     Trb << 1, 0, 0, W/2,    0, 1, 0, 0,     0, 0, 1, L/2,      0, 0, 0, 1;
    
//     Trans[0] = Tlf;
//     Trans[1] = Trf;
//     Trans[2] = Tlb;
//     Trans[3] = Trb;
    

//     return Trans;
// }

// std::vector<Eigen::Matrix4d> TransUpShoulder2RobotCenter_()
// {
//     double L(0.3610), W(0.094);// 240 80
//     Eigen::Matrix4d Tlf, Trf, Tlb, Trb;
//     std::vector<Eigen::Matrix4d> Trans(4);
    
//     Tlf << 1, 0, 0, W/2,    0, 1, 0, 0,     0, 0, 1, L/2,     0, 0, 0, 1;
//     Trf << 1, 0, 0, -W/2,    0, 1, 0, 0,     0, 0, 1, L/2,     0, 0, 0, 1;
//     Tlb << 1, 0, 0, W/2,    0, 1, 0, 0,     0, 0, 1, -L/2,      0, 0, 0, 1;
//     Trb << 1, 0, 0, -W/2,    0, 1, 0, 0,     0, 0, 1, -L/2,      0, 0, 0, 1;
    
//     Trans[0] = Tlf;
//     Trans[1] = Trf;
//     Trans[2] = Tlb;
//     Trans[3] = Trb;
    

//     return Trans;
// }

//FK

Eigen::Vector4d legFK(Eigen::Vector3d jointAngle, const int legParity){
	//jointAngle : (q1, q2, q3)
	//legParity : -1 if left, 1 if right
	
	Eigen::Matrix4d H1, H2, H3;
	H1 << cos(jointAngle[0]), -sin(jointAngle[0]), 0, -l1*legParity*cos(jointAngle[0]),
	      sin(jointAngle[0]), cos(jointAngle[0]),  0, -l1*legParity*sin(jointAngle[0]),
	      0, 0, 1, 0,
	      0, 0, 0, 1;
	H2 << 1, 0, 0, 0,
	      0, cos(jointAngle[1]), -sin(jointAngle[1]), -l2*cos(jointAngle[1]),
	      0, sin(jointAngle[1]), cos(jointAngle[1]), -l2*sin(jointAngle[1]),
	      0, 0, 0, 1;
	H3 << 1, 0, 0, 0,
	      0, cos(jointAngle[2]), -sin(jointAngle[2]), -l3*cos(jointAngle[2]),
	      0, sin(jointAngle[2]), cos(jointAngle[2]), -l3*sin(jointAngle[2]),
	      0, 0, 0, 1; 
	Eigen::Vector4d id(0,0,0,1);
	return H1*H2*H3*id;
}


//IK

Eigen::Vector3d legIK(Eigen::Vector4d legPosition, const int legParity, const int q3Parity){
	//legPosition : position of the end-effector with respect to each hip joint
	//legParity : -1 if left, 1 if right
	//q3Parity : decides L3 configuration
	
	double R1, R2, R3;
	double x(legPosition[0]), y(legPosition[1]), z(legPosition[2]);

	Eigen::Vector3d IKResult;	//IKResult : (q1, q2, q3)
	
	R1 = sqrt(pow(x, 2) + pow(y, 2) - pow(l1, 2));		//R1 : y2 distance from hip joint to end-effector
	R2 = sqrt(pow(R1, 2) + pow(z, 2)); 			//R2 : distance from hip joint to end-effector
	R3 = (pow(R2, 2)-pow(l2, 2)-pow(l3, 2))/(2*l2*l3);

	if(R3 > 1 || R3 < -1) (R3 > 0)?(R3 = 1):(R3 = -1);
	IKResult[2] = q3Parity*acos(R3);						// q3
	IKResult[1] = atan2(-z, R1)-atan2(l3*sin(IKResult[2]),l2+l3*cos(IKResult[2]));	// q2
	IKResult[0] = atan2(y, x)+atan2(R1, -l1*legParity);				// q1
	
	return IKResult; 
}


// Eigen::Vector3d LegIK(Eigen::Vector4d Lp)
// {
    
//     double D, F, G, H;
//     double x(Lp[0]), y(Lp[1]), z(Lp[2]);
//     Eigen::Vector3d theta;
    
//     if(pow(x, 2) + pow(y, 2) - pow(l1,2) < 0) //
//         F = l1;
//     else
//         F=sqrt(pow(x, 2) + pow(y, 2) - pow(l1,2));//


//     G = F;
//     H = sqrt(pow(G, 2) + pow(z, 2));

//     theta[0] = atan2(y, x) + atan2(F, -l1);//

//     D=(pow(H, 2) - pow(l2, 2) - pow(l3, 2))/(2 * l2 * l3);
//     if(-1 < D || D < 1)
//         theta[2] = acos(D);
//     else
//         theta[2] = 0;

    
//     theta[1] = atan2(-z,G) - atan2(l3 * sin(theta[2]), l2 + l3 * cos(theta[2]));
//     // std::cout << "legik result : " << theta << std::endl;
//     return theta;

// }

Eigen::VectorXd CalcIK(Eigen::Vector3d Center, Eigen::Vector3d Rotation, Eigen::Matrix4d LegPositon)
{
    Eigen::Matrix4d Iden4d;
    Iden4d << -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Eigen::VectorXd motorRadian(12);
    Eigen::Vector3d theta1, theta2, theta3, theta4; 
    
    


    Eigen::Vector4d Lp_UpShoulder1(TransRobotCenter2UpShoulder_1 * TransWorld2RobotCenter(Center, Rotation) * LegPositon.col(0)), 
                    Lp_UpShoulder2(TransRobotCenter2UpShoulder_2 * TransWorld2RobotCenter(Center, Rotation) * LegPositon.col(1)),
                    Lp_UpShoulder3(TransRobotCenter2UpShoulder_3 * TransWorld2RobotCenter(Center, Rotation) * LegPositon.col(2)),
                    Lp_UpShoulder4(TransRobotCenter2UpShoulder_4 * TransWorld2RobotCenter(Center, Rotation) * LegPositon.col(3));
    
    theta1 = legIK(Lp_UpShoulder1, -1, 1);
    theta2 = legIK(Lp_UpShoulder2, 1, 1);
    theta3 = legIK(Lp_UpShoulder3, -1, 1);
    theta4 = legIK(Lp_UpShoulder4, 1, 1 );
    
    
    motorRadian <<      theta1[0], theta1[1], theta1[2],
                        theta2[0], theta2[1], theta2[2],
                        theta3[0], theta3[1], theta3[2],
                        theta4[0], theta4[1], theta4[2];



    return motorRadian;

}


// std::vector<Eigen::Matrix4d> BodyIK(Eigen::Vector3d rot, Eigen::Vector3d center)
// {
//     Eigen::Matrix4d Rx, Ry, Rz, T, Rxyz_T, Tlf, Trf, Tlb, Trb;
//     double sHp(sin(pi/2)), cHp(cos(pi/2));
//     double L(0.3610), W(0.094);// 240 80
    
//     Rx <<   1,            0,              0,              0,
//             0,            cos(rot[0]),    -sin(rot[0]),   0,
//             0,            sin(rot[0]),    cos(rot[0]),    0,
//             0,            0,              0,              1;
    
//     Ry <<   cos(rot[1]),  0,              sin(rot[1]),    0,
//             0,            1,              0,              0,
//             -sin(rot[1]), 0,              cos(rot[1]),    0,
//             0,            0,              0,              1;
    
//     Rz <<   cos(rot[2]),  -sin(rot[2]),   0,              0,
//             sin(rot[2]),  cos(rot[2]),    0,              0,
//             0,            0,              1,              0,
//             0,            0,              0,              1;

//     T <<    0,  0,  0,  center[0],
//             0,  0,  0,  center[1],
//             0,  0,  0,  center[2],
//             0,  0,  0,  0;

//     Rxyz_T = Rz * (Ry * Rx) + T;
    
//     Tlf << cHp, 0, sHp, L/2,    0, 1, 0, 0,     -sHp, 0, cHp, W/2,     0, 0, 0, 1;
//     Trf << cHp, 0, sHp, L/2,    0, 1, 0, 0,     -sHp, 0, cHp, -W/2,     0, 0, 0, 1;
//     Tlb << cHp, 0, sHp, -L/2,    0, 1, 0, 0,     -sHp, 0, cHp, W/2,      0, 0, 0, 1;
//     Trb << cHp, 0, sHp, -L/2,    0, 1, 0, 0,     -sHp, 0, cHp, -W/2,      0, 0, 0, 1;
    
    
//     std::vector<Eigen::Matrix4d> Trans(4);
//     Trans[0]  = Rxyz_T * Tlf;
//     Trans[1]  = Rxyz_T * Trf;
//     Trans[2]  = Rxyz_T * Tlb;
//     Trans[3]  = Rxyz_T * Trb;
//     // std::cout << "bodyik result : " << Trans[0] << std::endl;

//     return Trans;

// }
    



// Eigen::VectorXd CalcIK(Eigen::Matrix4d RobotLp, Eigen::Vector3d angles, Eigen::Vector3d center)
// {
//     Eigen::Matrix4d Iden4d;
//     Iden4d << -1, 0, 0, 0,
//             0, 1, 0, 0,
//             0, 0, 1, 0,
//             0, 0, 0, 1;
//     Eigen::VectorXd motorRadian(12);

//     std::vector<Eigen::Matrix4d> trans = BodyIK(angles, center);
//     Eigen::Matrix4d Tlf(trans[0]), Trf(trans[1]), Tlb(trans[2]), Trb(trans[3]);
//     Eigen::Vector4d Lp1(Tlf.inverse() * RobotLp.row(0).transpose()), 
//                     Lp2(Iden4d * Trf.inverse() * RobotLp.row(1).transpose()),
//                     Lp3(Tlb.inverse() * RobotLp.row(2).transpose()),
//                     Lp4(Iden4d * Trb.inverse() * RobotLp.row(3).transpose());
    
//     motorRadian <<      LegIK(Lp1)[0], LegIK(Lp1)[1], LegIK(Lp1)[2],
//                         LegIK(Lp2)[0], LegIK(Lp2)[1], LegIK(Lp2)[2],
//                         LegIK(Lp3)[0], LegIK(Lp3)[1], LegIK(Lp3)[2],
//                         LegIK(Lp4)[0], LegIK(Lp4)[1], LegIK(Lp4)[2];


//     return motorRadian;

// }
