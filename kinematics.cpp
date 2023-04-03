#pragma once
#include "kinematics.h"

using namespace std;

#define Delta 0.0001	//极小值
	
HLRobot::HLRobot()
{
	//参数初始化
	gst = Isometry3d::Identity();

	w1 << 0, 0, 1;
	w2 << 0, 1, 0;
	w3 << 0, 1, 0;
	w4 << 0, 0, 1;
	w5 << 0, 1, 0;
	w6 << 0, 0, 1;

	q1 << 0, 0, Link_1;//1，2轴
	q2 << 0, 0, Link_1 + Link_2;//3轴
	qw << 0, 0, Link_1 + Link_2 + Link_3;//3，4，5轴上一点
	q6 << 0, 0, Link_1 + Link_2 + Link_3 + Link_4;//6轴上一点
	q7 << 1, 0, Link_1 + Link_2 + Link_3 + Link_4;//w6外一点

	theta << 0, 0, 0, 0, 0, 0;
	pose << 0, 0, 1475, 0, 0, -180;

	triPos_1 << 0, 0, 0, 0, 0, 0;
	triPos_2 << 0, 0, 0, 0, 0, 0; 
	triPos_3 << 0, 0, 0, 0, 0, 0; 
	cirCenterPos << 0, 0, 0, 0, 0, 0;

	gst_0 = Isometry3d::Identity();
	Zyz2Gst(pose, gst_0);
}

HLRobot::~HLRobot()
{
	
}

//正运动学，从关节坐标得到位姿，传入th单位为角度制
void HLRobot::RobotForward(Vector6D th)
{
	Isometry3d g_1, g_2, g_3, g_4, g_5, g_6;
	g_1 = Isometry3d::Identity();
	g_2 = Isometry3d::Identity();
	g_3 = Isometry3d::Identity();
	g_4 = Isometry3d::Identity();
	g_5 = Isometry3d::Identity();
	g_6 = Isometry3d::Identity();

	GetJointGst(q1, w1, th[0], g_1);
	GetJointGst(q1, w2, th[1], g_2);
	GetJointGst(q2, w3, th[2], g_3);
	GetJointGst(qw, w4, th[3], g_4);
	GetJointGst(qw, w5, th[4], g_5);
	GetJointGst(qw, w6, th[5], g_6);
	gst = g_1 * g_2 * g_3 * g_4 * g_5 * g_6 * gst_0;

	theta << th[0], th[1], th[2], th[3], th[4], th[5];
	
	Gst2Zyz(gst, pose);
}

void HLRobot::RobotBackward(Vector6D pos)
{
	//传入pos单位为毫米和角度制
	double th[6] = { 0 };
	Isometry3d g1, g2, g3;
	g1 = Isometry3d::Identity();
	g2 = Isometry3d::Identity();
	g3 = Isometry3d::Identity();

	Vector3d p, q, r, temp;
	p = VectorXd::Zero(3, 1);
	q = VectorXd::Zero(3, 1);
	r = VectorXd::Zero(3, 1);
	temp = VectorXd::Zero(3, 1);

	double delta = 0;

	Zyz2Gst(pos, gst);
	g1 = gst * gst_0.inverse();

	//计算theta3
	p = qw;
	q = q1;
	temp = g1 * p - q;
	r << 0, 0, Link_1 + Link_2;
	delta = temp.norm();
	SubProblem3(w3, p, q, r, delta, th[2]);
	/*cout << th[2] / M_PI * 180 << endl;*/

	//计算theta1,2
	Isometry3d g_3 = Isometry3d::Identity();
	GetJointGst(q2, w3, th[2] / M_PI * 180, g_3);
	p = g_3 * qw;
	q = g1 * qw;
	r = q1;
	SubProblem2(w1, w2, p, q, r, th[0], th[1]);
	/*cout << th[0] / M_PI * 180 << endl;
	cout << th[1] / M_PI * 180 << endl;*/

	//计算theta4,5
	Isometry3d g_1 = Isometry3d::Identity();
	Isometry3d g_2 = Isometry3d::Identity();
	GetJointGst(q1, w1, th[0] / M_PI * 180, g_1);
	GetJointGst(q1, w2, th[1] / M_PI * 180, g_2);
	g2 = g_3.inverse() * g_2.inverse() * g_1.inverse() * g1;
	p = q6;
	q = g2 * q6;
	r = qw;
	SubProblem2(w4, w5, p, q, r, th[3], th[4]);
	/*cout << th[3] / M_PI * 180 << endl;
	cout << th[4] / M_PI * 180 << endl;*/

	//计算theta6
	Isometry3d g_4 = Isometry3d::Identity();
	Isometry3d g_5 = Isometry3d::Identity();
	GetJointGst(qw, w4, th[3] / M_PI * 180, g_4);
	GetJointGst(qw, w5, th[4] / M_PI * 180, g_5);
	g3 = g_5.inverse() * g_4.inverse() * g2;
	p = q7;
	q = g3 * p;
	r = q6;
	SubProblem1(w6, p, q, r, th[5]);
	/*cout << th[5] / M_PI * 180 << endl;*/

	for (int i = 0; i < 6; i++) {
		theta[i] = th[i] * 180 / M_PI;
	}

	for (int i = 0; i < 6; i++) {
		pose[i] = pos[i];
	}

}
//SubProblem1—3得到的角度都是弧度制
void HLRobot::SubProblem1(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double &angle)
{
	Vector3d u = p - r;
	Vector3d v = q - r;
	Vector3d u_temp = u - omega * omega.dot(u);
	Vector3d v_temp = v - omega * omega.dot(v);
	//angle = atan2(omega.dot(u_temp.cross(v_temp)), (u_temp.dot(v_temp)));
	if ( u_temp.norm() > Delta ) {
		angle = atan2(omega.dot(u_temp.cross(v_temp)), (u_temp.dot(v_temp)));
	}
}

void HLRobot::SubProblem2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r, double &angle1, double &angle2)
{
	Vector3d u = p - r;
	Vector3d v = q - r;
	double alpha, beta, gama;
	double temp1, temp2;

	temp1 = pow((omega1.dot(omega2)), 2) - 1;
	alpha = (omega1.dot(omega2) * (omega2.dot(u)) - omega1.dot(v)) / temp1;
	beta = (omega1.dot(omega2) * (omega1.dot(v)) - omega2.dot(u)) / temp1;
	
	Vector3d w = omega1.cross(omega2);
	temp1 = u.dot(u) - pow(alpha, 2) - pow(beta, 2) - 2 * alpha * beta * omega1.dot(omega2);
	temp2 = temp1 / (w.dot(w));
	
	if (temp2 >= 0) {
		gama = -sqrt(temp2);
	} else {
		gama = 0;
	}

	Vector3d z = alpha * omega1 + beta * omega2 + gama * (omega1.cross(omega2));
	Vector3d c = r + z;
	SubProblem1(omega1, c, q, r, angle1);
	SubProblem1(omega2, p, c, r, angle2);
}

void HLRobot::SubProblem3(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double delta, double &angle)
{
	Vector3d u = p - r;
	Vector3d v = q - r;

	Vector3d u_temp = u - omega * omega.dot(u);
	Vector3d v_temp = v - omega * omega.dot(v);

	double delta_temp = sqrt(pow(delta, 2) - pow(omega.dot(p - q), 2));
	double t = (u_temp.dot(u_temp) + v_temp.dot(v_temp) - delta_temp * delta_temp) / (2 * u_temp.norm() * v_temp.norm());
	double th0 = atan2(omega.dot(u_temp.cross(v_temp)), (u_temp.dot(v_temp)));
	
	angle = th0 - acos(t);
}


//测试，th为角度制
void test(HLRobot &HLRobot, Vector6D &th, Vector6D &pos)
{
	// 前向
    cout << "Forward:" << endl;
	HLRobot.RobotForward(th);
	cout << HLRobot.pose << endl;

	// 后向
    cout << "Backward:" << endl;
	HLRobot.RobotBackward(pos);
	cout << HLRobot.theta << endl;
}


