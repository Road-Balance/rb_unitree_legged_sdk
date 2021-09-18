// KINEMATICS_RB.H
// Coordinate assignment : Every coordinate complies ROS assigned coordinates
// 			   forward -> x, left -> y, up -> z 
// Unit : SI units (ex : meter)
//



#ifndef _GLIBCXX_IOSTREAM
#include <iostream>
#endif

#ifndef _GLIBCXX_CMATH
#include <cmath>
#endif

#ifndef _GLIBCXX_VECTOR
#include <vector>
#endif

#ifndef EIGEN_CORE_H
#include <Eigen/Core>
#endif

#ifndef EIGEN_DENSE_H
#include <Eigen/Dense>
#endif

#ifndef RB_KINEMATICS_H
#define RB_KINEMATICS_H
#endif



#define PI 3.1415926
#define l1 0.0838
#define l2 0.2
#define l3 0.2
#define L 0.3610
#define W 0.094




double degToRad(double deg){
	double r = deg*PI/180;
	return r;
}
double radToDeg(double rad){
	double r = rad*180/PI;
	return r;
}


//FK and transformations

/*Eigen::Vector4d legFK(Eigen::Vector3d jointAngle, const int legParity){
	//jointAngle : (q1, q2, q3)
	//legParity : -1 if left, 1 if right
	//Homogeneous Transformations T10 ~ T32 should be applied in reverse of the following order (starts from T32)

	Eigen::Matrix4d T10, T21r, T21t, T32;	
	
	//T10 : forward transforamtion from {0}->{1} (on RViz, hip -> thigh_fixed)
	T10 << 1, 0, 0, 0,
	       0, cos(jointAngle[0]), -sin(jointAngle[0]), -l1*legParity*cos(jointAngle[0]),
	       0, sin(jointAngle[0]), cos(jointAngle[0]),  -l1*legParity*sin(jointAngle[0]),
	       0, 0, 0, 1;
	
	//T21r : forward transforamtion from {1}->{2}, rotation only (on Rviz, thigh_fixed -> thigh)
	T21r << cos(jointAngle[1]), 0, sin(jointAngle[1]), 0,
	                0,          1,         0,          0,
	        -sin(jointAngle[1]),0, cos(jointAngle[1]), 0,
	                0,          0,         0,          1;

	//T21t : forward transformation from {1}->{2}, translation only (on RViz, thigh -> calf)
	T21t << 1, 0, 0, 0,
	     	0, 1, 0, 0,
		0, 0, 1, -l2,
		0, 0, 0, 1;

	//T32 : forward transformation from {2}->{3} (on RViz, calf -> foot)
	T32 << cos(jointAngle[2]), 0, sin(jointAngle[2]), -l3*sin(jointAngle[2]),
	     	       0,          1,         0,                    0,
	       -sin(jointAngle[2]),0, cos(jointAngle[2]), -l3*cos(jointAngle[2]), 	
	               0,          0,         0,                    1;

	
	Eigen::Vector4d id(0,0,0,1);
	return T10*T21r*T21t*T32*id;
}*/

Eigen::Matrix4d invHT(Eigen::Matrix4d ht){
	Eigen::Matrix3d rotInv = (ht.block<3,3>(0,0)).transpose();	
	Eigen::Matrix4d ret = Eigen::MatrixXd::Identity(4,4);
	
	ret.block<3,3>(0,0) = rotInv;
	ret.block<3,1>(0,3) = -rotInv * ht.block<3,1>(0,3);

	return ret;
}

Eigen::Matrix4d bodyToHip(const int legID){
	
	const int FR = (legID/2)?(-1):(1);
	const int RL = (legID%2)?(1):(-1);
	Eigen::Matrix4d THB;

	THB << 1, 0, 0, FR*L/2,
	       0, 1, 0, RL*W/2,
	       0, 0, 1, 0, 
	       0, 0, 0, 1;
	
	return THB;
}

Eigen::Matrix4d hipToThigh(Eigen::Vector3d jointAngle, const int legID){
	Eigen::Matrix4d TTH;
	const int legParity = (legID%2)?(1):(-1);

	TTH << 1, 0, 0, 0,
	       0, cos(jointAngle[0]), -sin(jointAngle[0]), l1*legParity*cos(jointAngle[0]),
	       0, sin(jointAngle[0]), cos(jointAngle[0]),  l1*legParity*sin(jointAngle[0]),
	       0, 0, 0, 1;

	return TTH;
}


Eigen::Matrix4d thighToCalf(Eigen::Vector3d jointAngle){
	Eigen::Matrix4d TCT;

	TCT << cos(jointAngle[1]), 0, sin(jointAngle[1]), -l2*sin(jointAngle[1]),
	               0,          1,         0,          0,
	       -sin(jointAngle[1]),0, cos(jointAngle[1]), -l2*cos(jointAngle[1]),
	               0,          0,         0,          1;

	return TCT;
}

Eigen::Matrix4d calfToFoot(Eigen::Vector3d jointAngle){
	Eigen::Matrix4d TFC;

	TFC << cos(jointAngle[2]), 0, sin(jointAngle[2]), -l3*sin(jointAngle[2]),
	               0,          1,         0,          0,
	       -sin(jointAngle[2]),0, cos(jointAngle[2]), -l3*cos(jointAngle[2]),
	               0,          0,         0,          1;

	return TFC;
}

Eigen::Vector4d legFK(Eigen::Vector3d jointAngle, const int legID){
	Eigen::Vector4d id(0, 0, 0, 1);
	return hipToThigh(jointAngle, legID)*thighToCalf(jointAngle)*calfToFoot(jointAngle)*id;
}

Eigen::Vector4d bodyFK(Eigen::Vector3d jointAngle, const int legID){
	Eigen::Vector4d id(0, 0, 0, 1);
	return bodyToHip(legID)*hipToThigh(jointAngle, legID)*thighToCalf(jointAngle)*calfToFoot(jointAngle)*id;

}




//IK

Eigen::Vector3d legIK(Eigen::Vector4d legPosition, const int legParity, const int q3Parity){
	//legPosition : position of the end-effector with respect to each hip joint
	//legParity : -1 if left, 1 if right
	//q3Parity : decides L3 configuration
	
	double R1, R2, R3;
	double x(legPosition[0]), y(legPosition[1]), z(legPosition[2]);

	Eigen::Vector3d IKResult;	//IKResult : (q1, q2, q3)
	
	R1 = sqrt(pow(y, 2) + pow(z, 2) - pow(l1, 2));		//R1 : y2 distance from hip joint to end-effector
	R2 = sqrt(pow(R1, 2) + pow(x, 2)); 			//R2 : distance from hip joint to end-effector
	R3 = (pow(R2, 2)-pow(l2, 2)-pow(l3, 2))/(2*l2*l3);

	if(R3 > 1 || R3 < -1) R3 = floor(R3);
	IKResult[2] = q3Parity*acos(R3);						// q3
	IKResult[1] = atan2(-x, R1)-atan2(l3*sin(IKResult[2]),l2+l3*cos(IKResult[2]));	// q2
	IKResult[0] = atan2(z, y)+atan2(R1, -l1*legParity);				// q1
	
	return IKResult; 
}

Eigen::Vector3d bodyIK(Eigen::Vector4d bodyPosition, const int legID, const int q3Parity){
	
	Eigen::Vector4d mul(-1, -1, -1, 1);
	Eigen::Vector4d hipPosition((-1*hipFK(legID))*(mul.asDiagonal())*bodyPosition);
	const int legParity = (legID%2)?(1):(-1);

	return legIK(hipPosition, legParity, q3Parity);

}


//VK 
//VK will be used for trajectory generation... TBD
