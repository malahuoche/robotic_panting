#include <iostream>
#include "kinematics.h"
#include <vector>

using namespace std;

void GetCircle(HLRobot &HLRobot, vector<Vector6D> &posVec1)
{
	Vector6D posTemp;
	for (int i = 0; i < 37; i++)
	{
		posTemp << HLRobot.cirCenterPos[0] + 120 * cos(10 * i * M_PI / 180), HLRobot.cirCenterPos[1] + 120 * sin(10 * i * M_PI / 180), HLRobot.cirCenterPos[2], HLRobot.cirCenterPos[3], HLRobot.cirCenterPos[4], HLRobot.cirCenterPos[5];
		posVec1.push_back(posTemp);
	}
}

void GetTriangle(HLRobot &HLRobot, vector<Vector6D> &posVec2)
{
	Vector6D posTemp;
	//k为三角形边的编号，j为每条边点的编号，i为坐标编号
	for (int k = 0; k < 3; k++)
	{
		for (int j = 0; j < 6; j++)
		{
			for (int i = 0; i < 6; i++)
			{
				if (k == 0)
					posTemp(i) = (HLRobot.triPos_2(i) - HLRobot.triPos_1(i)) / (6 * 1.0) * j * 1.0 + HLRobot.triPos_1(i);
				else if (k == 1)
					posTemp(i) = (HLRobot.triPos_3(i) - HLRobot.triPos_2(i)) / (6 * 1.0) * j * 1.0 + HLRobot.triPos_2(i);
				else
					posTemp(i) = (HLRobot.triPos_1(i) - HLRobot.triPos_3(i)) / (6 * 1.0) * j * 1.0 + HLRobot.triPos_3(i);
			}
			posVec2.push_back(posTemp);
		}
	}
	//回到第一个顶点
	posVec2.push_back(HLRobot.triPos_1);
}
void GetCmd(HLRobot &HLRobot, vector<string> &robotCommand, string* cmdString, vector<Vector6D> pos_vec)
{
	string data2String[6], cmdTemp;

	for (int i = 0; i < pos_vec.size(); i++)
	{
		//计算关节角，存放在HLRobot对象的theta向量中
		HLRobot.RobotBackward(pos_vec[i]);

		for (int j = 0; j < 6; j++)
		{
			data2String[j] = to_string(HLRobot.theta(j));
			if (j == 0)
			{
				cmdTemp = cmdString[1];
				cmdTemp = cmdTemp + data2String[j] + cmdString[3];
			}
			else if (j != 5)
				cmdTemp = cmdTemp + data2String[j] + cmdString[3];
			else
				cmdTemp = cmdTemp + data2String[j] + cmdString[2];
		}
		robotCommand.push_back(cmdTemp);
		robotCommand.push_back(cmdString[0]);
	}

	for (int i = 0; i < robotCommand.size(); i++)
		cout << robotCommand[i] << endl;
}

void GetCircleCmd(HLRobot &HLRobot, vector<string> &robotCommand, vector<Vector6D> &posVec1)
{
	string cmdString[4], cmdTemp;
	cmdString[0] = "[2# Move.Joint joint_angel]";
	cmdString[1] = "[2# joint_angle=";
	cmdString[2] = "]";
	cmdString[3] = ",";

	GetCmd(HLRobot, robotCommand, cmdString, posVec1);
}

void GetTriangleCmd(HLRobot &HLRobot, vector<string> &robotCommand, vector<Vector6D> &posVec2)
{
	string cmdString[4], cmdTemp;
	cmdString[0] = "[2# Move.Line P1]";
	cmdString[1] = "[2# P1=";
	cmdString[2] = "]";
	cmdString[3] = ",";

	GetCmd(HLRobot, robotCommand, cmdString, posVec2);
}
