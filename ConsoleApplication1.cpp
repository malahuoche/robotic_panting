//// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。
////
//
//#include<iostream>
//#include<winsock.h>
//#include <conio.h>
//#include "ftp/FtpControl.h"
//#include "TrajectoryPlan.h"
//#include "eigen3/Eigen/Core"
//
//#pragma comment(lib,"ws2_32.lib")
//using namespace std;
//
//void initialization();
//#pragma comment(lib, "WS2_32.lib")
//
//int main()
//{   
//	//定义长度变量
//	int send_len = 0;
//	int recv_len = 0;
//	//定义发送缓冲区和接受缓冲区
//    char send_buf[100] = {};
//	char recv_buf[200] = {};
//    string recvstr;
//	//定义服务端套接字，接受请求套接字
//	SOCKET s_server;
//	//服务端地址客户端地址
//	SOCKADDR_IN server_addr;
//	initialization();
//	//填充服务端信息
//	server_addr.sin_family = AF_INET;
//	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
//	server_addr.sin_port = htons(2090);
//	//创建套接字
//	s_server = socket(AF_INET, SOCK_STREAM, 0);
//	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
//		cout << "服务器连接失败！" << endl;
//		WSACleanup();
//	}
//	else {
//		cout << "服务器连接成功！" << endl;
//	}
//
//	//登录
//    send_len = send(s_server, "[1# System.Login 0]", 100, 0);
//    recv_len = recv(s_server, recv_buf, 100, 0);
//    cout << recv_buf << endl;
//    memset(recv_buf,'\0',sizeof(recv_buf));
//    Sleep(1200);
//	//使能
//    send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
//    recv_len = recv(s_server, recv_buf, 200, 0);
//    recv_len = recv(s_server, recv_buf, 200, 0);
//    cout << recv_buf << endl;
//    memset(recv_buf, '\0', sizeof(recv_buf));
//    Sleep(1200);
//	//系统停止
//	send_len = send(s_server, "[0# System.Abort 1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(1200);
//	//启动
//	send_len = send(s_server, "[0# System.Start 1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(1200);
//	//Home
//	send_len = send(s_server, "[0# Robot.Home 1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(1200);
//	//自动模式
//	send_len = send(s_server, "[0# System.Auto 1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(1200);
//
//	FtpControl::Upload("192.168.10.101", "data", "data_line_ang1.txt", "severdata.txt");
//
//	//在此添加程序
//
//	//PPB使能
//	send_len = send(s_server, "[0# PPB Enable 1,1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << "[PPB_Enable]" << '\t' << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(200);
//	//关节坐标系
//	send_len = send(s_server, "[0# Robot.Frame 1,1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << "[Frame]" << '\t' << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(200);
//	//PPB读取data文件
//	send_len = send(s_server, "[0# PPB.ReadFile 1,/data/data.txt]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << "[ReadFile]" << '\t' << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(500);
//	//到达起始点
//	send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << "[ToStartPoint]" << '\t' << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(1000);
//	//PPB运行
//	send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
//	recv_len = recv(s_server, recv_buf, 100, 0);
//	cout << "[Run]" << '\t' << recv_buf << endl;
//	memset(recv_buf, '\0', sizeof(recv_buf));
//	Sleep(1000);
//
//
//	closesocket(s_server);
//	//释放DLL资源
//	WSACleanup();
//	return 0;
//}
//void initialization() {
//	//初始化套接字库
//	WORD w_req = MAKEWORD(2, 2);//版本号
//	WSADATA wsadata;
//	int err;
//	err = WSAStartup(w_req, &wsadata);
//	if (err != 0) {
//		cout << "初始化套接字库失败！" << endl;
//	}
//	else {
//		cout << "初始化套接字库成功！" << endl;
//	}
//	//检测版本号
//	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
//		cout << "套接字库版本号不符！" << endl;
//		WSACleanup();
//	}
//	else {
//		cout << "套接字库版本正确！" << endl;
//	}
//}
