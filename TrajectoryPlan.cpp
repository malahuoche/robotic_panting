#pragma once
#include "TrajectoryPlan.h"
using namespace std;

TrajectoryPlan::TrajectoryPlan()
{
	motionAcc = 0;
	motionVel = 0;
	motionTime = 0;
}

TrajectoryPlan::~TrajectoryPlan()
{

}
void TrajectoryPlan::Str2Piont(VectornD &coodinate, string dir)
{
	ifstream inFile;
	vector<double> viaZyzDou;
	string line;
	inFile.open(dir);
	
	while (getline(inFile, line)) {
		stringstream s(line);
		string str;
		while (getline(s, str, ',')) {
			viaZyzDou.push_back(stod(str));
		}
		s.clear();
	}
	inFile.close();
	
	if (viaZyzDou.size() % 6 == 0) {
		for (int i = 0; i < viaZyzDou.size() / 6; i++) {
			Vector6D temp;
			temp << viaZyzDou[i * 6], viaZyzDou[i * 6 + 1], viaZyzDou[i * 6 + 2], viaZyzDou[i * 6 + 3], viaZyzDou[i * 6 + 4], viaZyzDou[i * 6 + 5];
			coodinate.push_back(temp);
		}
	} else {
		cout << "Data format or number is error!" << endl;
	}
	/*for (int i = 0; i < coodinate.size(); i++) {
		cout << coodinate[i] << endl;
	}*/
}

void TrajectoryPlan::GetLineInfo()
{
	string dir = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\data_line.txt";
	Str2Piont(cornerPointZyz, dir);
	cout << cornerPointZyz.size() << endl;
}

void  TrajectoryPlan::MotionConfig(double acc, double time)
{
	if (acc > 0 && time > 0) {
		motionAcc = acc;
		motionTime = time;
	} else {
		cout << "acc or time is error!" << endl;
	}
}

void TrajectoryPlan::LineLFPB(bool isPos, double startPoint, double endPoint, string dir1, string dir2, bool &flag1, bool &flag2, bool end)
{
	ifstream inFile;
	ofstream outFile;
	double flag = endPoint - startPoint >= 0 ? 1.0 : -1.0;
	double acc = flag * motionAcc;
	double tb = 0;
	//cout << pow(acc * motionTime, 2) << endl;
	double temp = pow(acc * motionTime, 2) -  4 * acc * (endPoint - startPoint);
	if (temp > 0) {
		tb = 0.5 * (motionTime - sqrt(temp) / motionAcc);
		cout << "acc:" << acc << endl;
		cout << "tb:" << tb << "\n" << endl;
	} else {
		cout << "Increase Acc" << endl;
	}
	if (tb < 0) {
		return;
	}

	//文件为空，flag=1
	if (flag2 && !flag1) {
		inFile.open(dir1);
		outFile.open(dir2, ios::out | ios::trunc);
		flag1 = true;
		flag2 = false;
	} else if (!flag2 && flag1) {
		inFile.open(dir2);
		outFile.open(dir1, ios::out | ios::trunc);
		flag2 = true;
		flag1 = false;
	}
	//cout << flag1 << " " << flag2 << endl;
	
	if (inFile && outFile) {
		double sampleTime = 0;
		double data = 0;

		for (int i = 0; i < motionTime * 1000; i++) {
			//状态区分
			sampleTime = i / 1000.0;
			if (i >= 0 && i < tb * 1000.0) {
				//STATUS = ACCELERATION;
				data = startPoint + 0.5 * acc * pow(sampleTime, 2);
			} else if (i >= tb * 1000.0 && i < (motionTime - tb) * 1000) {
				//STATUS = CONSTANT_SPEED;
				data = startPoint + acc * tb * (sampleTime - 0.5 * tb);
			} else {
				//STATUS = DECELERATION;
				data = endPoint - 0.5 * acc * pow(motionTime - sampleTime, 2);
			}
			
			//cout << data << endl;
			string str;
			getline(inFile, str);
			if (end) {
				str += to_string(data) + "\n";
			} else if (isPos) {
				str += to_string(data) + ",\n";
			} else {
				str += to_string(data) + " \n";
			}
			outFile << str;
			
		}
		inFile.close();
		outFile.close();
	} else {
		cout << "ERROR!" << endl;
	}
}

void TrajectoryPlan::GetLineFile(bool isPos, Vector6D &startPoint, Vector6D &endPoint, string dir1, string dir2, bool &flag1, bool &flag2)
{
	LineLFPB(isPos, startPoint[0], endPoint[0], dir1, dir2, flag1, flag2, false);
	LineLFPB(isPos, startPoint[1], endPoint[1], dir1, dir2, flag1, flag2, false);
	LineLFPB(isPos, startPoint[2], endPoint[2], dir1, dir2, flag1, flag2, false);
	LineLFPB(isPos, startPoint[3], endPoint[3], dir1, dir2, flag1, flag2, false);
	LineLFPB(isPos, startPoint[4], endPoint[4], dir1, dir2, flag1, flag2, false);
	LineLFPB(isPos, startPoint[5], endPoint[5], dir1, dir2, flag1, flag2, true);
	flag1 = false;
	flag2 = true;
}

void TrajectoryPlan::GetLineFile(bool isPos)
{
	cout << "\n-------------------------Line-------------------------\n";

	string dir1, dir2;
	ofstream outFileAng;
	ofstream newFile;
	//dirResult = "F:\\robotHomework\\lab3_4\\Project2\\src\\data\\data_ang.txt";
	VectornD viaPointZyz;
	bool flag1 = false;
	bool flag2 = true;

	GetLineInfo();

	if (cornerPointZyz.size() < 2) {
		cout << "corner pionts error!" << endl;
		return;
	}
	for (int i = 0; i < cornerPointZyz.size() - 1; i++) {
		
		if (isPos) {
			dir1 = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\line\\data_line_zyz" + to_string(i + 1) + ".txt";
			dir2 = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\line\\data_line_ang" + to_string(i + 1) + ".txt";

			newFile.open(dir1, ios::out | ios::trunc);
			newFile.close();
			newFile.clear();

			GetLineFile(isPos, cornerPointZyz[i], cornerPointZyz[i + 1], dir1, dir2, flag1, flag2);

			Str2Piont(viaPointZyz, dir1);

			outFileAng.open(dir2, ios::out | ios::trunc);
			if (outFileAng) {
				for (int i = 0; i < viaPointZyz.size(); i++) {
					RobotBackward(viaPointZyz[i]);
					outFileAng << to_string(theta[0]) + " " + to_string(theta[1]) + " " + to_string(theta[2])
						+ " " + to_string(theta[3]) + " " + to_string(theta[4]) + " " + to_string(theta[5]) + "\n";
				}
				outFileAng.close();

				VectornD().swap(viaPointZyz);
			}
		} else {
			dir1 = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\line\\data_line_ang" + to_string(2 * i + 1) + ".txt";
			dir2 = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\line\\data_line_ang" + to_string(2 * i + 2) + ".txt";
			newFile.open(dir1, ios::out | ios::trunc);
			newFile.close();
			newFile.clear();

			GetLineFile(isPos, cornerPointZyz[i], cornerPointZyz[i + 1], dir1, dir2, flag1, flag2);
		}
	}
}

void TrajectoryPlan::GetCirInfo()
{
	ifstream inFile;
	string line, dir;
	dir = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\data_cir_shape.txt";
	inFile.open(dir);
	while (getline(inFile, line))
	{
		bool temp = true;
		stringstream s(line);
		string str;
		while (getline(s, str, ',')) {
			istringstream(str) >> temp;
			cirShape.push_back(temp);
		}
		s.clear();
	}
	if (cirShape.size() % 2 != 0) {
		cout << "cirShape error!" << endl;
		vector<bool>().swap(cirShape);
	}

	dir = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\data_cir.txt";
	Str2Piont(cirPointZyz, dir);
	if (cirPointZyz.size() % 3 != 0) {
		cout << "cirPointZyz error!" << endl;
		VectornD().swap(cirPointZyz);
	}

	/*for (int i = 0; i < cirPointZyz.size(); i++) {
		cout << cirPointZyz[i] << endl;
	}
	for (int i = 0; i < cirShape.size(); i++) {
		cout << cirShape[i] << endl;
	}*/
}

double* TrajectoryPlan::GetCirInfo(Vector6D pos1, Vector6D pos2, Vector6D pos3, bool flag1, bool flag2)
{
	double a = pos1[0] - pos2[0];
	double b = pos1[1] - pos2[1];
	double c = pos1[0] - pos3[0];
	double d = pos1[1] - pos3[1];
	double e = (pow(pos1[0], 2) - pow(pos2[0], 2) + pow(pos1[1], 2) - pow(pos2[1], 2)) * 0.5;
	double f = (pow(pos1[0], 2) - pow(pos3[0], 2) + pow(pos1[1], 2) - pow(pos3[1], 2)) * 0.5;

	double *info = new double [5];

	double centerX = (b * f - d * e) / (b * c - a * d);
	double centerY = (c * e - a * f) / (b * c - a * d);
	double r = sqrt(pow(pos1[0] - centerX, 2) + pow(pos1[1] - centerY, 2));

	if (flag1) {
		info[0] = acos((pos1[0] - centerX) / r);
	}
	else {
		info[0] = -acos((pos1[0] - centerX) / r);
	}
	if (flag2) {
		info[1] = acos((pos3[0] - centerX) / r);
	}
	else {
		info[1] = 2 * M_PI - acos((pos3[0] - centerX) / r);
	}

	info[2] = r;
	info[3] = centerX;
	info[4] = centerY;
	return info;
}

//th1为正，flag1为true；thf小于pi,flag2为true,逆时针取点
void TrajectoryPlan::CirLFPB(Vector6D pos1, Vector6D pos2, Vector6D pos3, string dir1, string dir2, bool flag1, bool flag2)
{
	double *info = GetCirInfo(pos1, pos2, pos3, flag1, flag2);
	double th0 = info[0] * 180 / M_PI;
	double thf = info[1] * 180 / M_PI;
	if (thf <= th0) {
		delete[] info;
		cout << "th0 & thf:" << th0 << " " << thf << endl;
		return;
	} else {
		double r = info[2];
		double x0 = info[3];
		double y0 = info[4];
		delete[] info;
		cout << "circle info:" << th0 << " " << thf << " " << r << " " << x0 << " " << y0 << endl;

		double tb = 0;
		//cout << pow(motionAcc * motionTime, 2) << endl;
		double temp = pow(motionAcc * motionTime, 2) - 4 * motionAcc * (thf - th0);
		if (temp > 0) {
			tb = 0.5 * (motionTime - sqrt(temp) / motionAcc);
			if (tb < 0) {
				return;
			} else {
				cout << "acc:" << motionAcc << endl;
				cout << "tb:" << tb << "\n" << endl;
			}
		} else {
			cout << "Increase Acc" << endl;
		}

		double sampleTime = 0;
		double data = 0;
		ofstream outFile1, outFile2;
		Vector6D viaPos;

		viaPos << pos1;
		outFile1.open(dir1, ios::out | ios::trunc);
		outFile2.open(dir2, ios::out | ios::trunc);
		if (outFile1 && outFile2) {
			for (int i = 0; i < motionTime * 1000.0; i++) {
				//状态区分
				sampleTime = i / 1000.0;
				if (i >= 0 && i < tb * 1000.0) {
					//STATUS = ACCELERATION;
					data = th0 + 0.5 * motionAcc * pow(sampleTime, 2);
				}
				else if (i >= tb * 1000.0 && i < (motionTime - tb) * 1000.0) {
					//STATUS = CONSTANT_SPEED;
					data = th0 + motionAcc * tb * (sampleTime - 0.5 * tb);
				}
				else {
					//STATUS = DECELERATION;
					data = thf - 0.5 * motionAcc * pow(motionTime - sampleTime, 2);
				}
				viaPos[0] = x0 + r * cos(data / 180 * M_PI);
				viaPos[1] = y0 + r * sin(data / 180 * M_PI);
				//cout << viaPos << endl;
				RobotBackward(viaPos);
				outFile1 << to_string(theta[0]) + " " + to_string(theta[1]) + " " + to_string(theta[2])
					+ " " + to_string(theta[3]) + " " + to_string(theta[4]) + " " + to_string(theta[5]) + "\n";
				outFile2 << to_string(viaPos[0]) + " " + to_string(viaPos[1]) + " " + to_string(viaPos[2])
					+ " " + to_string(viaPos[3]) + " " + to_string(viaPos[4]) + " " + to_string(viaPos[5]) + "\n";
			}
			outFile1.close();
		}
	}
}

void TrajectoryPlan::GetCirFile() 
{
	cout << "\n------------------------Circle------------------------\n";

	GetCirInfo();

	if (cirPointZyz.size() < 3 || cirShape.size() < 2) {
		return;
	}
	string dir1, dir2;
	for (int i = 0; i < cirPointZyz.size() / 3; i++) {
		dir1 = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\cir\\data_cir_ang" + to_string(i + 1) + ".txt";
		dir2 = "C:\\Users\\Administrator\\Desktop\\ConsoleApplication1\\ConsoleApplication1\\data\\cir\\data_cir_zyz" + to_string(i + 1) + ".txt";
		
		CirLFPB(cirPointZyz[i * 3], cirPointZyz[i * 3 + 1], cirPointZyz[i * 3 + 2], dir1, dir2, cirShape[i * 2], cirShape[i * 2 + 1]);
	}
}