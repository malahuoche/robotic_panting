#include <vector>
#include <sstream>
#include <fstream>
#include "kinematics.h"
using namespace std;

typedef vector<Vector6D> VectornD;

class TrajectoryPlan : public HLRobot {
public:
	TrajectoryPlan();
	~TrajectoryPlan();
	
	void MotionConfig(double acc, double time);//设置加速度和时间
	void GetLineFile(bool isPos);//获取关节角点位文件
	void GetCirFile();

protected:
	void Str2Piont(VectornD &coodinate, string dir);//从txt获取viapoint笛卡尔坐标/关节坐标

	void GetLineInfo();//获取直线端点
	void LineLFPB(bool isPos, double startPos, double endPos, string dir1, string dir2, bool &flag1, bool &flag2, bool end);//两点,一维
	void GetLineFile(bool isPos, Vector6D &startpos, Vector6D &endPos, string dir1, string dir2, bool &flag1, bool &flag2);//获取笛卡尔点位文件

	void GetCirInfo();//获取圆弧信息
	double* GetCirInfo(Vector6D pos1, Vector6D pos2, Vector6D pos3, bool flag1, bool flag2);//计算圆心、半径、起始角、结束角
	void CirLFPB(Vector6D pos1, Vector6D pos2, Vector6D pos3, string dir1, string dir2, bool flag1, bool flag2);//三点

private:
	VectornD cornerPointZyz;
	VectornD cirPointZyz;
	vector<bool> cirShape;
	double motionAcc;
	double motionVel;
	double motionTime;
};