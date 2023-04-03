#pragma once
//#include "TrajectoryPlan.h"
#include "lab3cmd.h"
using namespace std;

int main()
{

//	TrajectoryPlan motion;
//	double acc = 5;
//	double time = 5;
//	motion.MotionConfig(acc, time);
//	motion.GetCirFile();
//	motion.GetLineFile(false);
	HLRobot HLRobot;
//
//	Vector6D th, pos;
//	th << -27.84, 2.947, 78.849, -0.033, 97.072, 147.849;
//	pos << 415.746, -219.622, 920.636, -29.489, 178.867, -33.751;
//	test(HLRobot, th, pos);
//
	vector<string> robotCommand;
	vector<Vector6D> posVec1, posVec2;

	HLRobot.cirCenterPos << 423.661, 2.115, 884.491, -8.288, 178.867, -33.751;
	GetCircle(HLRobot,posVec1);
	GetCircleCmd(HLRobot, robotCommand, posVec1);
	HLRobot.triPos_1 << 415.746, -219.622, 920.636, -29.489, 178.867, -33.751;
	HLRobot.triPos_2 << 555.893, -54.405, 920.636, -8.288, 178.867, -33.751;
	HLRobot.triPos_3 << 374.286, 208.621, 884.491, -8.287, 178.867, -33.751;
	GetTriangle(HLRobot, posVec2);
	GetTriangleCmd(HLRobot, robotCommand, posVec2);
	return 0;
}

