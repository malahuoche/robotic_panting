#pragma once
#include "TrajectoryPlan.h"
//#include "lab3cmd.h"
using namespace std;

int main()
{
	//课程设计
	TrajectoryPlan motion;
	double acc = 5;
	double time = 5;
	motion.MotionConfig(acc, time);
	//motion.GetCirFile();
	motion.GetLineFile(false);
	
	//HLRobot HLRobot;
	////正逆解测试
	//Vector6D th, pos;
	//th << -27.84, 2.947, 78.849, -0.033, 97.072, 147.849;
	//pos << 415.746, -219.622, 920.636, -29.489, 178.867, -33.751;
	//test(HLRobot, th, pos);

	//vector<string> robotCommand;
	//vector<Vector6D> posVec1, posVec2;
	////设置圆心
	//HLRobot.cirCenterPos << 423.661, 2.115, 884.491, -8.288, 178.867, -33.751;
	//GetCircle(HLRobot,posVec1);
	///*for (int i = 0; i < 37; i++)
	//	cout << posVec1[i] << endl;*/
	//GetCircleCmd(HLRobot, robotCommand, posVec1);

	////设置三角形3个顶点
	//HLRobot.triPos_1 << 415.746, -219.622, 920.636, -29.489, 178.867, -33.751;
	//HLRobot.triPos_2 << 555.893, -54.405, 920.636, -8.288, 178.867, -33.751;
	//HLRobot.triPos_3 << 374.286, 208.621, 884.491, -8.287, 178.867, -33.751;
	//GetTriangle(HLRobot, posVec2);
	///*for (int i = 0; i < posVec2.size(); i++)
	//	cout << posVec2[i] << endl;*/
	//GetTriangleCmd(HLRobot, robotCommand, posVec2);
	return 0;
}

