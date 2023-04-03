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
	
	void MotionConfig(double acc, double time);//���ü��ٶȺ�ʱ��
	void GetLineFile(bool isPos);//��ȡ�ؽڽǵ�λ�ļ�
	void GetCirFile();

protected:
	void Str2Piont(VectornD &coodinate, string dir);//��txt��ȡviapoint�ѿ�������/�ؽ�����

	void GetLineInfo();//��ȡֱ�߶˵�
	void LineLFPB(bool isPos, double startPos, double endPos, string dir1, string dir2, bool &flag1, bool &flag2, bool end);//����,һά
	void GetLineFile(bool isPos, Vector6D &startpos, Vector6D &endPos, string dir1, string dir2, bool &flag1, bool &flag2);//��ȡ�ѿ�����λ�ļ�

	void GetCirInfo();//��ȡԲ����Ϣ
	double* GetCirInfo(Vector6D pos1, Vector6D pos2, Vector6D pos3, bool flag1, bool flag2);//����Բ�ġ��뾶����ʼ�ǡ�������
	void CirLFPB(Vector6D pos1, Vector6D pos2, Vector6D pos3, string dir1, string dir2, bool flag1, bool flag2);//����

private:
	VectornD cornerPointZyz;
	VectornD cirPointZyz;
	vector<bool> cirShape;
	double motionAcc;
	double motionVel;
	double motionTime;
};