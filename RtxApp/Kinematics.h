#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <tchar.h>
#include <wchar.h>
#include <cmath>



class Kinematics
{
public:

	Kinematics();

	void updateConfig();

	bool IK (const Eigen::Matrix4d& COM, const Eigen::Matrix4d& left_leg, const Eigen::Matrix4d& right_leg);

	bool FK();

	bool FK (double l_leg_actual_angles[6], double r_leg_actual_angles[6], double waist_theta[2]);

	void GC (bool leftLeg);

	void setCurrentJointAngle (double l_leg_angles[6], double r_leg_angles[6]);

	void printState();

	void printState_v2();
	// note: must use const and & when passing eigen types to functions or the compile will occur error
	Eigen::Matrix3d getDHRot (const Eigen::Matrix4d& dh);

	Eigen::Vector3d getDHPos (const Eigen::Matrix4d& dh);

	Eigen::Matrix4d matDH (double a, double alpha, double d, double theta);
	
	Eigen::Vector3d RoTtoRPY (const Eigen::Matrix3d& rot);

	Eigen::Matrix3d RPYtoRot (const Eigen::Vector3d& rpy);

	Eigen::Matrix4d T_r_base[7];
	Eigen::Matrix4d T_l_base[7];
	Eigen::Matrix4d T_r_relative[7];
	Eigen::Matrix4d T_l_relative[7];
	
	Eigen::Vector3d l_hip_actual_pos, l_knee_actual_pos, l_ankle_actual_pos, l_foot_actual_pos;
	Eigen::Vector3d r_hip_actual_pos, r_knee_actual_pos, r_ankle_actual_pos, r_foot_actual_pos;

	// command angles
	double l_leg_target_angles[6];
	double r_leg_target_angles[6];
	double waist_target_angles[2];

protected:

	inline void updateDH();

	Eigen::Matrix4d pose[13];

	Eigen::Matrix4d T_BN[14];
	Eigen::Matrix4d T_C[14];

	// read from encoder
	double l_leg_actual_angle[6];
	double r_leg_actual_angle[6];
	double waist_actual_angle[2];

	// dh matrixs
	Eigen::Matrix4d	T_trunk_waist,
					T_waist_left_hip, T_waist_right_hip, T_left_hip_waist, T_right_hip_waist,
					T_trunk_left_hip, T_trunk_right_hip, T_trunk_1, T_trunk_r;
	
	// gravity compensate
	double link_cog[12];
	Eigen::Vector3d force[12];
	double mass[13];


	double pelvis_w;
	double pelvis_h;
	double shin;
	double thigh;
	double sole;

	// trash
	Eigen::Matrix3d R01, R02, R03, R05;

	Eigen::Matrix4d T01, T02, T03, T04, T05, T06, T12, T23, T34, T45, T56, 
								  T_waist_hip, T_trunk_hip;
};

#endif
