#include <iostream>
#include "robot_math.h"

using namespace Eigen;

#define Link_1 0.491
#define Link_2 0.450
#define Link_3 0.450
#define Link_4 0.084

class HLRobot {
public:
	HLRobot();
	~HLRobot();

	//3个子问题
	void SubProblem1(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double &angle1);
	void SubProblem2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r, double &angle1, double &angle2);
	void SubProblem3(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double delta, double &angle1);
	//正逆运动学
	void RobotForward(Vector6D th);
	void RobotBackward(Vector6D pos);

private:
	Vector3d qw, q2, q1, q6, q7;//单位为米
	Vector3d w1, w2, w3, w4, w5, w6;

public:
	Isometry3d gst_0, gst;
	Vector6D theta;//关节坐标，单位为角度制
	Vector6D pose;//笛卡尔坐标，单位为毫米、角度制
	Vector6D triPos_1, triPos_2, triPos_3, cirCenterPos;//三角形和圆形相关点的笛卡尔坐标，单位为毫米、角度制
};

void test(HLRobot &HLRobot, Vector6D &th, Vector6D &pos);

