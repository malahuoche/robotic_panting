#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "robot_math.h"

using namespace std;

#define Delta 0.000001	//极小值

//指数转换矩阵计算（R矩阵）,th为弧度制
void Exp(Vector3d w, double th, Matrix3d &output)
{
	double s = sin(th);
	double c = cos(th);
	double v = 1 - c;

	output <<
		w(0) * w(0) * v + c, w(0)* w(1)* v - w(2) * s, w(0)* w(2)* v + w(1) * s,
		w(0)* w(1)* v + w(2) * s, w(1)* w(1)* v + c, w(1)* w(2)* v - w(0) * s,
		w(0)* w(2)* v - w(1) * s, w(1)* w(2)* v + w(0) * s, w(2)* w(2)* v + c;
}

//关节转换矩阵，得到g矩阵，theta为角度制
void GetJointGst(Vector3d q, Vector3d w, double theta, Isometry3d &T)
{
	Matrix3d R = MatrixXd::Identity(3, 3);
	Vector3d p = VectorXd::Zero(3, 1);
	GetJointGst(q, w, theta, R, p);
	T.rotate(R);
	T.pretranslate(p);
	/*cout << T.matrix() << endl;*/
}
//theta为角度制
void GetJointGst(Vector3d q, Vector3d w, double theta, Matrix3d &R, Vector3d &p)
{
	Vector3d v = -w.cross(q);
	Matrix3d I = MatrixXd::Identity(3, 3);
	double th = theta / 180 * M_PI;
	if (w.norm() <= Delta) {
		//norm范数，w为0是认为是纯平移
		R = I;
	}
	else {
		Exp(w, th, R);
	}
	p = (I - R)* w.cross(v) + w * w.dot(v) * th;
}

//传入的pos中xyz单位为毫米，rpy角单位为角度制
void Zyz2Gst(Vector6D &pos, Isometry3d &gst)
{

	AngleAxisd rotate_roll(pos[5] * M_PI / 180, Vector3d(0, 0, 1));
	AngleAxisd rotate_pitch(pos[4] * M_PI / 180, Vector3d(0, 1, 0));
	AngleAxisd rotate_yaw(pos[3] * M_PI / 180, Vector3d(0, 0, 1));

	gst = Isometry3d::Identity();
	gst.rotate(rotate_yaw);
	gst.rotate(rotate_pitch);
	gst.rotate(rotate_roll);

	Vector3d p = VectorXd::Zero(3, 1);
	p(0) = pos[0] / 1000;
	p(1) = pos[1] / 1000;
	p(2) = pos[2] / 1000;
	
	gst.pretranslate(p);
}

void Gst2Zyz(Isometry3d &g, Vector6D &pos)
{
	pos[0] = g(0, 3) * 1000;
	pos[1] = g(1, 3) * 1000;
	pos[2] = g(2, 3) * 1000;

	double beta = atan2(sqrt(pow(g(2, 0), 2) + pow(g(2, 1), 2)), g(2, 2));
	pos[4] = beta  * 180 / M_PI;

	if (abs(sin(beta)) < Delta) {
		pos[3] = 0;
		if (abs(beta) < Delta) {
			pos[5] = atan2(-g(0, 1), g(0, 0)) * 180 / M_PI;
		}
		else {
			pos[5] = atan2(g(0, 1), -g(0, 0)) * 180 / M_PI;
		}
	} else {
		pos[3] = atan2(g(1, 2), g(0, 2)) * 180 / M_PI;
		pos[5] = atan2(g(2, 1), -g(2, 0)) * 180 / M_PI;
	}
}
