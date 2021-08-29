#include <iostream>
#include <cmath>
#include <Eigen/Dense> 
#include <vector>

// #include "pbPlots.hpp"
// #include "supportLib.hpp"

const double pi = M_PI; 
const Eigen::Matrix4d Iden4d = Eigen::Matrix<double, 4, 4>::Identity();

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

int BinomialFactor(int n, int k)
{
    return Factorial(n) / (Factorial(k) * Factorial(n -k));
}

double BezierCurve(double t, int k, double point)
{
    int n = 9;
    return point * BinomialFactor(n, k) * pow(t, (double)k) * pow((1 - t), (double)(n - k));
}

Eigen::Vector3d CalculateBezierStance(double phi_st, double V, double angle)
{   
    double c = cos(Deg2rad(angle));
    double s = sin(Deg2rad(angle));

    double A = 0.005;
    double halfStance = 0.05;
    double p_Stance = halfStance * (1 - 2 * phi_st);
    
    Eigen::Vector3d StancePosition(c * p_Stance * fabs(V),                       // stanceX                  
                                   - s * p_Stance * fabs(V),                     // stanceY                       
                                   - A * cos(pi / (2 * halfStance) * p_Stance)); // stanceZ

    // std::cout <<  p_Stance << std::endl;
    // *stancePtr = c * p_Stance * fabs(V); // stanceX
    // *(stancePtr + 1) = - s * p_Stance * fabs(V); // stanceY
    // *(stancePtr + 2) = - A * cos(pi / (2 * halfStance) * p_Stance); //stanceZ

    return StancePosition;
}

Eigen::Vector3d CalculateBezierSwing(double phi_sw, double V, double angle)
{
    double c = cos(Deg2rad(angle));
    double s = sin(Deg2rad(angle));

    Eigen::VectorXd X_(10), Y_(10), Z_(10), X(10), Y(10), Z(10); 

    // X_ <<    0.112, -0.165, -0.198, -0.198, -0.198, 0.0, 0.0, 0.0, 0.198, 0.198, 0.158, 0.112; // (12,1)
    // Y_ <<    0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05, 0.0, 0.0; 
    // Z_ <<    -0.310, -0.310, -0.231, -0.231, -0.231, -0.244, -0.244, -0.211, -0.211, -0.211, -0.310, -0.310;
    
    X_ <<    -0.05, -0.06, -0.07, -0.07, 0.0, 0.0, 0.07, 0.07, 0.06, 0.05; // (10,1)
    Y_ <<    0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05; 
    Z_ <<    0.0, 0.0, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06, 0.0, 0.0;

    X = X_ * (fabs(V) * c);
    Y = Y_ * (fabs(V) * s);
    Z = Z_ * fabs(V);

    Eigen::Vector3d SwingPositon(0, 0, 0);

    for(int i = 0; i < 10; i++ )
    {
        SwingPositon[0] += BezierCurve(phi_sw, i, X[i]); // swingX
        SwingPositon[1] += BezierCurve(phi_sw, i, Y[i]); // swingY
        SwingPositon[2] += BezierCurve(phi_sw, i, Z[i]); // swingZ
    }

    return SwingPositon;
}


// Kinematics
std::vector<Eigen::Matrix4d> BodyIK(Eigen::Vector3d rot, Eigen::Vector3d center)
{
    Eigen::Matrix4d Rx, Ry, Rz, T, Rxyz_T, Tlf, Trf, Tlb, Trb;
    double sHp(sin(pi/2)), cHp(cos(pi/2));
    double L(360.0), W(80.0);
    
    Rx <<   1,            0,              0,              0,
            0,            cos(rot[0]),    -sin(rot[0]),   0,
            0,            sin(rot[0]),    cos(rot[0]),    0,
            0,            0,              0,              1;
    
    Ry <<   cos(rot[1]),  0,              sin(rot[1]),    0,
            0,            1,              0,              0,
            -sin(rot[1]), 0,              cos(rot[1]),    0,
            0,            0,              0,              1;
    
    Rz <<   cos(rot[2]),  -sin(rot[2]),   0,              0,
            sin(rot[2]),  cos(rot[2]),    0,              0,
            0,            0,              1,              0,
            0,            0,              0,              1;

    T <<    0,  0,  0,  center[0],
            0,  0,  0,  center[1],
            0,  0,  0,  center[2],
            0,  0,  0,  0;

    Rxyz_T = Rz * (Ry * Rx) + T;
    
    Tlf << cHp, 0, sHp, L/2,    0, 1, 0, 0,     -sHp, 0, cHp, W/2,     0, 0, 0, 1;
    Trf << cHp, 0, sHp, L/2,    0, 1, 0, 0,     -sHp, 0, cHp, -W/2,     0, 0, 0, 1;
    Tlb << cHp, 0, sHp, -L/2,    0, 1, 0, 0,     -sHp, 0, cHp, W/2,      0, 0, 0, 1;
    Trb << cHp, 0, sHp, -L/2,    0, 1, 0, 0,     -sHp, 0, cHp, -W/2,      0, 0, 0, 1;
    
    
    std::vector<Eigen::Matrix4d> Trans(4);
    Trans[0]  = Rxyz_T * Tlf;
    Trans[1]  = Rxyz_T * Trf;
    Trans[2]  = Rxyz_T * Tlb;
    Trans[3]  = Rxyz_T * Trb;
    // std::cout << "bodyik result : " << Trans[0] << std::endl;

    return Trans;

}
    


Eigen::Vector3d LegIK(Eigen::Vector4d Lp)
{
    double l1(80), l2(100), l3(210), l4(220);
    double D, F, G, H;
    double x(Lp[0]), y(Lp[1]), z(Lp[2]);
    Eigen::Vector3d theta;
    
    F=sqrt(pow(x, 2) + pow(y, 2) - pow(l1,2));
    // if(F)
    //     F = l1;

    G = F - l2;
    H = sqrt(pow(G, 2) + pow(z, 2));

    theta[0] = -atan2(y, x) - atan2(F, -l1);

    D=(pow(H, 2) - pow(l3, 2) - pow(l4, 2))/(2 * l3 * l4);

    theta[2] = acos(D);
    // if(theta[2])
    //     theta[2] = 0;
    
    theta[1] = atan2(z,G) - atan2(l4 * sin(theta[2]), l3 + l4 * cos(theta[2]));
    // std::cout << "legik result : " << theta << std::endl;
    return theta;

}

Eigen::VectorXd CalcIK(Eigen::Matrix4d RobotLp, Eigen::Vector3d angles, Eigen::Vector3d center)
{
    // double omega(angles[0]), phi(angles[1]), psi(angles[2]);
    // double xm(center[0]), ym(center[1]), zm(center[2]);
    Eigen::VectorXd motorRadian(12);

    std::vector<Eigen::Matrix4d> trans = BodyIK(angles, center);
    std::cout << trans[0] << std::endl;
    Eigen::Matrix4d Tlf(trans[0]), Trf(trans[1]), Tlb(trans[2]), Trb(trans[3]);
    Eigen::Vector4d Lp1(Tlf.inverse() * RobotLp.row(0).transpose()), 
                    Lp2(Iden4d * Trf.inverse() * RobotLp.row(1).transpose()),
                    Lp3(Tlb.inverse() * RobotLp.row(2).transpose()),
                    Lp4(Iden4d * Trb.inverse() * RobotLp.row(3).transpose());
    
    
    motorRadian <<      LegIK(Lp1)[0], LegIK(Lp1)[1], LegIK(Lp1)[2],
                        LegIK(Lp2)[0], LegIK(Lp2)[1], LegIK(Lp2)[2],
                        LegIK(Lp3)[0], LegIK(Lp3)[1], LegIK(Lp3)[2],
                        LegIK(Lp4)[0], LegIK(Lp4)[1], LegIK(Lp4)[2];


    return motorRadian;

}