#include "Kinematics.h"

Kinematics::Kinematics()
{
	for (int i = 0; i < 13; ++i)
	{
		pose[i] = Eigen::Matrix4d::Identity();
	}
	for (int i = 0; i < 7; ++i)
	{
		T_r_base[i] = Eigen::Matrix4d::Identity();
		T_l_base[i] = Eigen::Matrix4d::Identity();
		T_r_relative[i] = Eigen::Matrix4d::Identity();
		T_l_relative[i] = Eigen::Matrix4d::Identity();
	}
	
	T_trunk_waist = Eigen::Matrix4d::Identity();
	T_waist_left_hip = Eigen::Matrix4d::Identity();
    T_waist_right_hip = Eigen::Matrix4d::Identity();

	for (int i = 0; i < 12; ++i)
	{
		link_cog[i] = 0;
	}
	for (int i = 0; i < 6; ++i)
	{
		r_leg_target_angles[i] = 0;
		l_leg_target_angles[i] = 0;
	}

	waist_actual_angle[0] = 0;
	waist_actual_angle[1] = 0;

	pelvis_w = 12.0;
	pelvis_h = 11.0;
	thigh = 26.0;
	shin = 22.0;
	sole = 12.5;

	mass[0] = 3.39;
	mass[1] = 0;
	mass[2] = 4.097;
	mass[3] = 4.58;
	mass[4] = 0;
	mass[5] = 3.684;
	mass[6] = 46.8;
	mass[7] = 3.684;
	mass[8] = 0;
	mass[9] = 4.58;
	mass[10] = 4.097;
	mass[11] = 0;
	mass[12] = 3.39;

}

bool Kinematics::IK(const Eigen::Matrix4d& COM, const Eigen::Matrix4d& left_leg, const Eigen::Matrix4d& right_leg)
{
	Eigen::Matrix4d l_pelvis = COM*matDH(pelvis_w, M_PI / 2, 0.0, 0);
	Eigen::Matrix4d l_hip = l_pelvis*matDH(0.0, -M_PI / 2, pelvis_h, 0)*matDH(0.0, -M_PI / 2, 0.0, -M_PI / 2);
	Eigen::Matrix4d l_ankle = left_leg*((matDH(0.0, M_PI / 2, 0.0, 0)*matDH(0.0, M_PI / 2, 0.0, -M_PI / 2 + 0)*matDH(0.0, 0, -sole, M_PI / 2)).inverse());
	Eigen::Vector3d dis = l_hip.block<3, 1>(0, 3) - l_ankle.block<3, 1>(0, 3);
	double absDis = sqrt(dis(0)*dis(0) + dis(1)* dis(1) + dis(2)*dis(2) );
	double CosineAbsDis = ((thigh*thigh + shin*shin - absDis*absDis) / (2 * thigh*shin));
	if (CosineAbsDis >= 1)
		l_leg_target_angles[3] = M_PI - acos(1.0);
	else if (((thigh*thigh + shin*shin - absDis*absDis) / (2 * thigh*shin)) <= -1)
		l_leg_target_angles[3] = M_PI - acos(-1.0);
	else
		l_leg_target_angles[3] = M_PI - acos(CosineAbsDis);
	
	Eigen::Vector3d l_ankle_x;
	l_ankle_x << -absDis, 0, 0;

	// asin(theta) + bcos(theta) = c
	// -->    R*sin(theta+phase) = c
	dis = getDHRot(l_ankle).inverse()*dis;

	double L2 = sqrt(l_ankle_x(0)*l_ankle_x(0) + l_ankle_x(1)* l_ankle_x(1));
	double phase2 = atan2(l_ankle_x(1), -l_ankle_x(0));
	double l_leg_target_angles4_total = 0;
	double SineX = dis(1) / L2;
	if (SineX >= 1)
		l_leg_target_angles4_total = asin(1.0) - phase2;
	else if (SineX <= -1)
		l_leg_target_angles4_total = asin(-1.0) - phase2;
	else
		l_leg_target_angles4_total = asin(SineX) - phase2;

	double l_leg_target_angles4_offset = 0;
	double CosineThigh = ((absDis*absDis + shin*shin - thigh*thigh) / (2 * absDis*shin));
	if (CosineThigh >= 1)
		l_leg_target_angles4_offset = acos(1.0);
	else if (CosineThigh <= -1)
		l_leg_target_angles4_offset = acos(-1.0);
	else
		l_leg_target_angles4_offset = acos(CosineThigh);

	l_leg_target_angles[4] = l_leg_target_angles4_total - l_leg_target_angles4_offset;


	double L1 = sqrt((cos(l_leg_target_angles4_total)*l_ankle_x(0) + sin(l_leg_target_angles4_total)*l_ankle_x(1))*(cos(l_leg_target_angles4_total)*l_ankle_x(0) + sin(l_leg_target_angles4_total)*l_ankle_x(1))+ l_ankle_x(2)*l_ankle_x(2));
	double phase1 = atan2(cos(l_leg_target_angles4_total)*l_ankle_x(0) + sin(l_leg_target_angles4_total)*l_ankle_x(1), l_ankle_x(2));
	double SineY = dis(0) / L1;
	if(SineY >= 1)
		l_leg_target_angles[5] = asin(1.0) - phase1;
	else if(SineY <= -1)
		l_leg_target_angles[5] = asin(-1.0) - phase1;
	else
		l_leg_target_angles[5] = asin(SineY) - phase1;


	Eigen::Matrix4d l_knee = left_leg*((matDH(shin, 0.0, 0.0, l_leg_target_angles[3]) * matDH(0.0, M_PI / 2, 0.0, l_leg_target_angles[4]) * matDH(0.0, M_PI / 2, 0.0, -M_PI / 2 + l_leg_target_angles[5])*matDH(0.0, 0, -sole, M_PI / 2)).inverse());
	Eigen::Matrix3d l_rot = getDHRot(l_hip).inverse()*getDHRot(l_knee);
	l_leg_target_angles[0] = -atan2(l_rot(1, 2), l_rot(2, 2));
	l_leg_target_angles[1] = atan2(-l_rot(0,2), -l_rot(1,2)/sin(l_leg_target_angles[0]));//asin(-l_rot(0, 2));
	l_leg_target_angles[2] = -atan2(l_rot(0, 1), l_rot(0, 0));


	Eigen::Matrix4d r_pelvis = COM*matDH(-pelvis_w, M_PI / 2, 0.0, 0);
	Eigen::Matrix4d r_hip = r_pelvis*matDH(0.0, -M_PI / 2, pelvis_h, 0)*matDH(0.0, -M_PI / 2, 0.0, -M_PI / 2);
	Eigen::Matrix4d r_ankle = right_leg*((matDH(0.0, M_PI / 2, 0.0, 0)*matDH(0.0, M_PI / 2, 0.0, -M_PI / 2 + 0)*matDH(0.0, 0, -sole, M_PI / 2)).inverse());
	Eigen::Vector3d r_dis = r_hip.block<3, 1>(0, 3) - r_ankle.block<3, 1>(0, 3);
	double r_absDis = sqrt(r_dis(0)*r_dis(0) + r_dis(1)* r_dis(1) + r_dis(2)*r_dis(2));
	double r_CosineAbsDis = ((thigh*thigh + shin*shin - r_absDis*r_absDis) / (2 * thigh*shin));
	if (r_CosineAbsDis >= 1)
		r_leg_target_angles[3] = M_PI - acos(1.0);
	else if (((thigh*thigh + shin*shin - r_absDis*r_absDis) / (2 * thigh*shin)) <= -1)
		r_leg_target_angles[3] = M_PI - acos(-1.0);
	else
		r_leg_target_angles[3] = M_PI - acos(r_CosineAbsDis);

	Eigen::Vector3d r_ankle_x;
	r_ankle_x << -r_absDis, 0, 0;

	// asin(theta) + bcos(theta) = c
	// -->    R*sin(theta+phase) = c
	r_dis = getDHRot(r_ankle).inverse()*r_dis;

	double R2 = sqrt(r_ankle_x(0)*r_ankle_x(0) + r_ankle_x(1)* r_ankle_x(1));
	double r_phase2 = atan2(r_ankle_x(1), -r_ankle_x(0));
	double r_leg_target_angles4_total = 0;
	double r_SineX = r_dis(1) / R2;
	if (r_SineX >= 1)
		r_leg_target_angles4_total = asin(1.0) - r_phase2;
	else if (r_SineX <= -1)
		r_leg_target_angles4_total = asin(-1.0) - r_phase2;
	else
		r_leg_target_angles4_total = asin(r_SineX) - r_phase2;

	double r_leg_target_angles4_offset = 0;
	double r_CosineThigh = ((r_absDis*r_absDis + shin*shin - thigh*thigh) / (2 * r_absDis*shin));
	if (r_CosineThigh >= 1)
		r_leg_target_angles4_offset = acos(1.0);
	else if (r_CosineThigh <= -1)
		r_leg_target_angles4_offset = acos(-1.0);
	else
		r_leg_target_angles4_offset = acos(r_CosineThigh);

	r_leg_target_angles[4] = r_leg_target_angles4_total - r_leg_target_angles4_offset;


	double R1 = sqrt((cos(r_leg_target_angles4_total)*r_ankle_x(0) + sin(r_leg_target_angles4_total)*r_ankle_x(1))*(cos(r_leg_target_angles4_total)*r_ankle_x(0) + sin(r_leg_target_angles4_total)*r_ankle_x(1)) + r_ankle_x(2)*r_ankle_x(2));
	double r_phase1 = atan2(cos(r_leg_target_angles4_total)*r_ankle_x(0) + sin(r_leg_target_angles4_total)*r_ankle_x(1), r_ankle_x(2));
	double r_SineY = r_dis(0) / R1;
	if (r_SineY >= 1)
		r_leg_target_angles[5] = asin(1.0) - r_phase1;
	else if (r_SineY <= -1)
		r_leg_target_angles[5] = asin(-1.0) - r_phase1;
	else
		r_leg_target_angles[5] = asin(r_SineY) - r_phase1;


	Eigen::Matrix4d r_knee = right_leg*((matDH(shin, 0.0, 0.0, r_leg_target_angles[3]) * matDH(0.0, M_PI / 2, 0.0, r_leg_target_angles[4]) * matDH(0.0, M_PI / 2, 0.0, -M_PI / 2 + r_leg_target_angles[5])*matDH(0.0, 0, -sole, M_PI / 2)).inverse());
	Eigen::Matrix3d r_rot = getDHRot(r_hip).inverse()*getDHRot(r_knee);
	r_leg_target_angles[0] = -atan2(r_rot(1, 2), r_rot(2, 2));
	r_leg_target_angles[1] = atan2(-r_rot(0,2), -r_rot(1,2)/sin(r_leg_target_angles[0]));//asin(-r_rot(0, 2));
	r_leg_target_angles[2] = -atan2(r_rot(0, 1), r_rot(0, 0));

	if(COM(1,3)+12 < left_leg(1,3))
	{
		l_leg_target_angles[1] *= -1;
		l_leg_target_angles[5] *= -1;
	}
	if(COM(1,3)-12 < right_leg(1,3))
	{
		r_leg_target_angles[1] *= -1;
		r_leg_target_angles[5] *= -1;
	}
	return true;
}

bool Kinematics::pre_FK(const Eigen::Matrix4d& T_cog, double *waist_angles)
{
	// compute T_waist_center from T_cog according to waist_angles command

	waist_target_angles[0] = waist_angles[0];
	waist_target_angles[1] = waist_angles[1];

	T_waist_yaw = matDH(0.0, 0, 0.0,  M_PI / 2+waist_angles[0]);
	T_waist_roll = matDH(0.0, M_PI / 2, 0.0, waist_angles[1]);

	T_waist_center = T_cog * T_waist_yaw * T_waist_roll;

	return true;
}

bool Kinematics::FK()
{
	updateDH();

	R01 = getDHRot(T_trunk_1);
	R02 = getDHRot(T02);
	R03 = getDHRot(T03);
	R05 = getDHRot(T05);

	l_hip_actual_pos = getDHPos(T02);
	l_knee_actual_pos = getDHPos(T03);
	l_ankle_actual_pos = getDHPos(T05);

	return true;
}

bool Kinematics::FK (const Eigen::Matrix4d& T_cog, double l_leg_actual_angles[6], double r_leg_actual_angles[6], double waist_theta[2])
{

	T_trunk_waist = matDH(0.0, M_PI / 2, 0.0,  M_PI / 2+waist_theta[0]);

	T_r_relative[0] = matDH(-pelvis_w, M_PI / 2, 0.0, waist_theta[1]);
	T_r_relative[1] = matDH(0.0, -M_PI / 2, pelvis_h, r_leg_actual_angles[0]);
	T_r_relative[2] = matDH(0.0, -M_PI / 2, 0.0, -M_PI / 2 + r_leg_actual_angles[1]);
	T_r_relative[3] = matDH(thigh, 0.0, 0.0, r_leg_actual_angles[2]);
	T_r_relative[4] = matDH(shin, 0.0, 0.0, r_leg_actual_angles[3]);
	T_r_relative[5] = matDH(0.0, M_PI / 2, 0.0, r_leg_actual_angles[4]);
	Eigen::Matrix4d tmp1 = matDH(0.0, M_PI / 2, 0.0, -M_PI / 2 + r_leg_actual_angles[5]);
	T_r_relative[6] = tmp1*matDH(0.0, 0, -sole, M_PI / 2);


	T_l_relative[0] = matDH(pelvis_w, M_PI / 2, 0.0, waist_theta[1]);
	T_l_relative[1] = matDH(0.0, -M_PI / 2, pelvis_h, l_leg_actual_angles[0]);
	T_l_relative[2] = matDH(0.0, -M_PI / 2, 0.0, -M_PI / 2 + l_leg_actual_angles[1]);
	T_l_relative[3] = matDH(thigh, 0.0, 0.0, l_leg_actual_angles[2]);
	T_l_relative[4] = matDH(shin, 0.0, 0.0, l_leg_actual_angles[3]);
	T_l_relative[5] = matDH(0.0, M_PI / 2, 0.0, l_leg_actual_angles[4]);
	Eigen::Matrix4d tmp2 = matDH(0.0, M_PI / 2, 0.0, -M_PI / 2+ l_leg_actual_angles[5]);
	T_l_relative[6] = tmp2*matDH(0.0, 0, -sole, M_PI / 2 );

	T_r_base[0] = T_cog*T_trunk_waist*T_r_relative[0];
	for (int i = 1; i < 7; ++i)
	{
		T_r_base[i] = T_r_base[i - 1] * T_r_relative[i];
	}

	T_l_base[0] = T_cog*T_trunk_waist*T_l_relative[0];
	for (int i = 1; i < 7; ++i)
	{
		T_l_base[i] = T_l_base[i - 1] * T_l_relative[i];
	}

	//l_hip_actual_pos = getDHPos(T_l_relative[2]);
	//r_hip_actual_pos = getDHPos(T_r_relative[2]);

	//l_knee_actual_pos = getDHPos(T_l_relative[3]);
	//r_knee_actual_pos = getDHPos(T_r_relative[3]);

	//l_ankle_actual_pos = getDHPos(T_l_relative[5]);
	//r_ankle_actual_pos = getDHPos(T_r_relative[5]);

	//l_foot_actual_pos = getDHPos(T_l_relative[6]);
	//r_foot_actual_pos = getDHPos(T_r_relative[6]);

	//

	return true;
}

void Kinematics::GC (int sup_leg, double *l_leg_gc, double *r_leg_gc)
{
	Eigen::Vector3d force[12];
	Eigen::Vector3d e[12];
	Eigen::Vector3d zVector;
	Eigen::Vector3d CoG[12];
	Eigen::Vector3d r;
	Eigen::Vector3d g;
	Eigen::Vector3d nVector;
	Eigen::Matrix4d T_BN[14];
	Eigen::Matrix4d T_C[14];

	for(int i=0; i<7;i++)
	{
		T_C[i] = T_l_relative[6-i].inverse();
		T_BN[i] = T_l_base[6-i].inverse();
		T_C[i+7] = T_r_relative[i];
		T_BN[i+7] = T_r_base[i];
	}


	zVector << 0, 0, 1;
	g << 0, 0, -9.8;

	if (sup_leg == 0)
	{
		for (int i = 0; i<12; i++)
		{
			CoG[i] = (getDHPos(T_BN[i+1]) + getDHPos(T_BN[i + 2]))/2;
			force[i] = mass[i + 1] * g;
			e[i] = getDHRot(T_C[i+1])*g;
		}
		for (int i = 0; i<12; i++)
		{
			nVector = T_C[i+1].block<3, 1>(0, 2);
			for (int j = i; j<12; j++)
			{
				r = CoG[j] - getDHPos(T_BN[i+1]);
				r = r - nVector*nVector.dot(r);
				link_cog[i] = link_cog[i] + e[i].dot(r.cross(force[i]));
			}
		}
	}
	else if (sup_leg == 1)
	{
		for (int i = 11; i>=0; i--)
		{
			CoG[i] = (getDHPos(T_BN[i]) + getDHPos(T_BN[i - 1]))/2;
			force[i] = mass[i - 1] * g;
			e[i] = getDHRot(T_C[i]).inverse()*g;
		}
		for (int i = 11; i>=0; i--)
		{
			nVector = T_C[i].block<3, 1>(0, 2);
			for (int j = i; j<12; j++)
			{
				r = CoG[j] - getDHPos(T_BN[i]);
				r = r - nVector*nVector.dot(r);
				link_cog[i] = link_cog[i] + e[i].dot(r.cross(force[i]));
			}
		}
	}

	/*for(int i=0; i<6; i++){
		l_leg_gc[i] = link_cog[5-i];
		r_leg_gc[i] = link_cog[6+i];
	}*/
	if (sup_leg == 0){
		l_leg_gc[1] = 2 * M_PI / 180;
		l_leg_gc[5] = 1 * M_PI / 180;
		r_leg_gc[1] = 0;
		r_leg_gc[5] = 0;

		//l_leg_gc[1] = 1000;
		//l_leg_gc[5] = 1000;
		//r_leg_gc[1] = 0;
		//r_leg_gc[5] = 0;
	}
	else{
		l_leg_gc[1] = 0;
		l_leg_gc[5] = 0;
		r_leg_gc[1] = -2 * M_PI / 180;
		r_leg_gc[5] = -1 * M_PI / 180;

		//l_leg_gc[1] = 0;
		//l_leg_gc[5] = 0;
		//r_leg_gc[1] = -1000;
		//r_leg_gc[5] = -1000;
	}
}

inline void Kinematics::updateDH()
{
	T_trunk_waist = matDH(0.0, M_PI/2, 0.0, M_PI/2 + waist_actual_angle[0]);
	T_waist_hip = matDH(pelvis_w, -M_PI/2, 0.0, waist_actual_angle[1]);
	T01 = matDH(0.0, M_PI/2, -pelvis_h, l_leg_actual_angle[0]);
	T12 = matDH(0.0, -M_PI/2, 0.0, -M_PI/2 + l_leg_actual_angle[1]);
	T23 = matDH(thigh, 0.0, 0.0, l_leg_actual_angle[2]);
	T34 = matDH(shin, 0.0, 0.0, l_leg_actual_angle[3]);
	T45 = matDH(0.0, M_PI/2, 0.0, l_leg_actual_angle[4]);
	// matDH(0.0, pi/2, -pelvis_h, config->l_leg_actual_angle[0], config->T01); ankle to foot

	T_trunk_hip = T_trunk_waist*T_waist_hip;
	T_trunk_1 = T_trunk_hip*T01;
	T02 = T_trunk_1*T12;
	T03 = T02*T23;
	T04 = T03*T34;
	T05 = T04*T45;
}

void Kinematics::printState_v2()
{
  printf("%f %f %f \n", T_BN[13](0,3), T_BN[13](1,3), T_BN[13](2,3));
}

void Kinematics::printState()
{
  printf("left ankle pos: ");
  for ( int i = 0; i < 3; i++)
	  printf("%f ", l_ankle_actual_pos);
  printf("\n");
}

void Kinematics::setCurrentJointAngle(double l_leg_angles[6], double r_leg_angles[6])
{
	for (int i = 0; i < 6; i++)
	{
		memcpy(&l_leg_actual_angle[i], &l_leg_angles[i], sizeof(l_leg_angles[0]));
		memcpy(&r_leg_actual_angle[i], &r_leg_angles[i], sizeof(r_leg_angles[0]));
	}
}

Eigen::Matrix3d Kinematics::getDHRot (const Eigen::Matrix4d& dh)
{
	Eigen::Matrix3d rot;
	rot = dh.block<3, 3>(0, 0);
	return rot;
}

Eigen::Vector3d Kinematics::getDHPos (const Eigen::Matrix4d& dh)
{
	Eigen::Vector3d pos;
	pos = dh.block<3, 1>(0, 3);
	return pos;
}


//Eigen::Matrix4d Kinematics::matDH (double a, double alpha, double d, double theta)
//{
//	// standard version 2016/6/3
//	Eigen::Matrix4d dh;
//	dh(0, 0) = cos(theta);			  dh(0, 1) = -sin(theta);			dh(0, 2) = 0.0;			dh(0, 3) = a;
//	dh(1, 0) = sin(theta)*cos(alpha); dh(1, 1) = cos(theta)*cos(alpha); dh(1, 2) = -sin(alpha); dh(1, 3) = -d*sin(alpha);
//	dh(2, 0) = sin(theta)*sin(alpha); dh(2, 1) = cos(theta)*sin(alpha); dh(2, 2) = cos(alpha);  dh(2, 3) = d*cos(alpha);
//	dh(3, 0) = 0.0;					  dh(3, 1) = 0.0;                   dh(3, 2) = 0.0;         dh(3, 3) = 1.0;
//
//	return dh;
//}

Eigen::Matrix4d Kinematics::matDH(double a, double alpha, double d, double theta)
{
	Eigen::Matrix4d dh;
	dh(0, 0) = cos(theta); dh(0, 1) = -sin(theta)*cos(alpha); dh(0, 2) = sin(theta)*sin(alpha);  dh(0, 3) = a*cos(theta);
	dh(1, 0) = sin(theta); dh(1, 1) = cos(theta)*cos(alpha);  dh(1, 2) = -cos(theta)*sin(alpha); dh(1, 3) = a*sin(theta);
	dh(2, 0) = 0.0;		   dh(2, 1) = sin(alpha);			  dh(2, 2) = cos(alpha);			 dh(2, 3) = d;
	dh(3, 0) = 0.0;        dh(3, 1) = 0.0;                    dh(3, 2) = 0.0;                    dh(3, 3) = 1.0;

	return dh;
}

Eigen::Vector3d Kinematics::RoTtoRPY (const Eigen::Matrix3d& rot)
{
	Eigen::Vector3d rpy;
	rpy(0) = atan2(rot(2,1), rot(2,2));
	rpy(1) = atan2(-rot(2, 0), sqrt(rot(2, 1)*rot(2, 1)+rot(2, 2)*rot(2, 2)));
	rpy(2) = atan2(rot(1,0), rot(0,0));

	return rpy;
}

Eigen::Matrix3d Kinematics::RPYtoRot (const Eigen::Vector3d& rpy)
{
	Eigen::Matrix3d rot;
	//rot = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
	return rot;
}