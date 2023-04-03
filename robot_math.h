#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

#define M_PI  3.14159265358979323846264338327950   // pi
typedef Eigen::Matrix<double, 1, 6> Vector6D;

void Exp(Vector3d w, double th, Matrix3d &R);

void GetJointGst(Vector3d q, Vector3d w, double theta, Isometry3d &T);
void GetJointGst(Vector3d q, Vector3d w, double theta, Matrix3d &R, Vector3d &p);

void Gst2Zyz(Isometry3d &g, Vector6D &pos);
void Zyz2Gst(Vector6D &pos, Isometry3d &gst);