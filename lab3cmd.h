#include <iostream>
#include "kinematics.h"
#include <vector>
using namespace std;

void GetCircle(HLRobot &HLRobot, vector<Vector6D> &posVec1);
void GetTriangle(HLRobot &HLRobot, vector<Vector6D> &posVec2);
void GetCmd(HLRobot &HLRobot, vector<string> &robotCommand, string* cmdString, vector<Vector6D> pos_vec);
void GetCircleCmd(HLRobot &HLRobot, vector<string> &robotCommand, vector<Vector6D> &posVec1);
void GetTriangleCmd(HLRobot &HLRobot, vector<string> &robotCommand, vector<Vector6D> &posVec2);