// File:          jpking_lqr.cpp
// Date:          2022.10.17
// Description:   基于lqr的控制方法
// Author:        xuzhanhao
// Modifications: 
#include <iostream>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Brake.hpp>
#include <webots/InertialUnit.hpp>
#include "WheeledBiped.hpp"
#include <webots/InertialUnit.hpp>
#include <webots/keyboard.hpp>
#include <fstream>
#include <stdio.h>
// 添加1完毕

using namespace webots;
using namespace std;
double theta2 = 0.5236;//大腿小腿夹角，目前为30度
double l_bar = 0.3*sin(theta2/2)*2;  // 名义腿长
double M = 4.97;  // 除轮以外等效质量[kg]
double m = 0.15708; // 轮子质量[kg]
double g = 9.8; // [m / s ^ 2]
double l_datui = 0.3;   //大腿长度[m]
double l_xiaotui = 0.3;  //小腿长度[m]
double R_wheel = 0.05;// 轮子半径[m]
double D = 0.30 + 2 * 0.015 + 2 * 0.03 + 0.02; //两轮之间的距离
double damping = 0.01;
double I_wheel = m * R_wheel * R_wheel / 2;
double I_jiti_x = M * (0.26 * 0.26 + 0.13 * 0.13) / 12;
double I_jiti_z = M * (0.26 * 0.26 + 0.13 * 0.13) / 12;
double I_jiti_y = M * (0.13 * 0.13 + 0.13 * 0.13) / 12;//!!!忽略了腿部的重量
double t = 0.0; //时钟
const double epsilon = 0.0001;

double angle,anglevelocity, yawvelocity;//倾斜角
double m11, m12, m22, m33;//质量矩阵声明
//相关动力学参数计算
double a_dyna = R_wheel * (M + 2 * m + 2 * I_wheel / R_wheel / R_wheel);
double b_dyna = M * R_wheel * l_bar;
double c_dyna = I_jiti_z + M * l_bar * l_bar;
double d_dyna = M * g * l_bar;
double e_dyna = M * l_bar;
double f_dyna = 1 / (R_wheel * (m * D + I_wheel * D / R_wheel / R_wheel + 2 * I_jiti_y / D));
//状态空间方程参数计算
double A23 = (0 - b_dyna * d_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double A43 = (    a_dyna * d_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double B21 = (c_dyna + b_dyna )    / (a_dyna * c_dyna - b_dyna * e_dyna);
double B41 = (0 - e_dyna-a_dyna)   / (a_dyna * c_dyna - b_dyna * e_dyna);
double B61 = f_dyna;
double B62 = -f_dyna;

//定义Q\R
vector<vector<double>> Q = {
				{100.0,  0.0,      0.0,    0.0},
				{0.0,  1.0,    0.0,    0.0},
				{0.0,  0.0,      1.0,    0.0},
				{0.0,  0.0,      0.0,    10.0}
};
vector<vector<double>> R = { {10,0},{0,10} };
//定义目标的参数
double x_target = 3;
double v_target = 0;
double angle_target = 0.19185;  //这个值应该是随着机器人质心的位置而改变的
double anglevle_target = 0;
double yaw_target = 0;




//创建矩阵的函数
vector<vector<double>> creatmatrix(int h, int l)
{
	vector<vector<double>> v;
	for (int i = 0; i < h; i++)
	{
		vector<double>v1(l, 0);
		v.push_back(v1);
	}
	return v;
}
//矩阵乘常数并返回
vector<vector<double>> multiply_num(const vector<vector<double>>& A, double num)
{
	int A_h = A.size();
	int A_l = A[0].size();
	vector<vector<double>> B = creatmatrix(A_h, A_l);
	for (int i = 0; i < A_h; i++)
	{
		for (int j = 0; j < A_l; j++)
		{
			B[i][j] = num * A[i][j];
		}
	}
	return B;
}
//矩阵A+矩阵B=矩阵C，并返回
vector<vector<double>> plusmatrix(const vector<vector<double>>& A, const vector<vector<double>>& B)
{
	int h = A.size();
	int l = A[0].size();
	vector<vector<double>> C;
	C = creatmatrix(h, l);

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < l; j++)
		{
			C[i][j] = A[i][j] + B[i][j];
			if (abs(C[i][j]) < epsilon)
			{
				C[i][j] = 0.0;
			}
		}
	}
	return C;
}
//矩阵A*矩阵B=矩阵C，并返回
vector<vector<double>> multiply(const vector<vector<double>>& A, const vector<vector<double>>& B)
{
	int A_h = A.size();
	int A_l = A[0].size();
	int B_h = B.size();
	int B_l = B[0].size();
	if (A_l != B_h)
	{
		cout << "两矩阵维数无法相乘" << endl;
		exit(0);
	}
	vector<vector<double>> C = creatmatrix(A_h, B_l);
	for (int i = 0; i < A_h; i++)
	{
		for (int j = 0; j < B_l; j++)
		{
			C[i][j] = 0;
			for (int k = 0; k < A_l; k++)
			{
				C[i][j] += A[i][k] * B[k][j];
			}
			if (abs(C[i][j]) < epsilon)
			{
				C[i][j] = 0.0;
			}
			//cout<<C[i][j]<<"\t";
		}
		//cout<<endl;
	}
	return C;
}
//矩阵A-矩阵B=矩阵C，并返回
vector<vector<double>> minusmatrix(const vector<vector<double>>& A, const vector<vector<double>>& B)
{
	int h = A.size();
	int l = A[0].size();
	vector<vector<double>> C;
	C = creatmatrix(h, l);

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < l; j++)
		{
			C[i][j] = A[i][j] - B[i][j];
			if (abs(C[i][j]) < epsilon)
			{
				C[i][j] = 0.0;
			}
		}
	}
	return C;
}
//矩阵A*矩阵B*矩阵C=目标矩阵，并返回
vector<vector<double>> multiply_tri(const vector<vector<double>>& A, const vector<vector<double>>& B,
const vector<vector<double>>& C)
{
	vector<vector<double>> D, E;
	D = multiply(A, B);
	E = multiply(D, C);
	return E;
}
//输入矩阵A，输出矩阵A的转置矩阵AT
vector<vector<double>> trans(const vector<vector<double>>& A)
{
	vector<vector<double>> AT = creatmatrix(A[0].size(), A.size());
	int h = AT.size();
	int l = AT[0].size();
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < l; j++)
		{
			AT[i][j] = A[j][i];
		}
	}
	return AT;
}
//输入矩阵A,输出矩阵A的逆矩阵inv_A
vector<vector<double>> inverse(const vector<vector<double>>& A)
{
	if (A.size() != A[0].size())
	{
		cout << "输入矩阵维数不合法" << endl;
		exit(0);
	}
	int n = A.size();
	vector<vector<double>> inv_A = creatmatrix(n, n);
	vector<vector<double>> L = creatmatrix(n, n);
	vector<vector<double>> U = creatmatrix(n, n);
	vector<vector<double>> inv_L = creatmatrix(n, n);
	vector<vector<double>> inv_U = creatmatrix(n, n);
	//LU分解
		//L矩阵对角元素为1
	for (int i = 0; i < n; i++)
	{
		L[i][i] = 1;
	}
	//U矩阵第一行
	for (int i = 0; i < n; i++)
	{
		U[0][i] = A[0][i];
	}
	//L矩阵第一列
	for (int i = 1; i < n; i++)
	{
		L[i][0] = 1.0 * A[i][0] / A[0][0];
	}

	//计算LU上下三角
	for (int i = 1; i < n; i++)
	{
		//计算U（i行j列）
		for (int j = i; j < n; j++)
		{
			double tem = 0;
			for (int k = 0; k < i; k++)
			{
				tem += L[i][k] * U[k][j];
			}
			U[i][j] = A[i][j] - tem;
			if (abs(U[i][j]) < epsilon)
			{
				U[i][j] = 0.0;
			}
		}
		//计算L（j行i列）
		for (int j = i; j < n; j++)
		{
			double tem = 0;
			for (int k = 0; k < i; k++)
			{
				tem += L[j][k] * U[k][i];
			}
			L[j][i] = 1.0 * (A[j][i] - tem) / U[i][i];
			if (abs(L[i][j]) < epsilon)
			{
				L[i][j] = 0.0;
			}
		}

	}
	//L U剩余位置设为0
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (i > j)
			{
				U[i][j] = 0.0;
			}
			else if (i < j)
			{
				L[i][j] = 0.0;
			}
		}
	}
	//LU求逆
	//求矩阵U的逆 
	for (int i = 0; i < n; i++)
	{
		inv_U[i][i] = 1 / U[i][i];// U对角元素的值，直接取倒数
		for (int k = i - 1; k >= 0; k--)
		{
			double s = 0;
			for (int j = k + 1; j <= i; j++)
			{
				s = s + U[k][j] * inv_U[j][i];
			}
			inv_U[k][i] = -s / U[k][k];//迭代计算，按列倒序依次得到每一个值，
			if (abs(inv_U[k][i]) < epsilon)
			{
				inv_U[k][i] = 0.0;
			}
		}
	}
	//求矩阵L的逆
	for (int i = 0; i < n; i++)
	{
		inv_L[i][i] = 1; //L对角元素的值，直接取倒数，这里为1
		for (int k = i + 1; k < n; k++)
		{
			for (int j = i; j <= k - 1; j++)
			{
				inv_L[k][i] = inv_L[k][i] - L[k][j] * inv_L[j][i];
				if (abs(inv_L[k][i]) < epsilon)
				{
					inv_L[k][i] = 0.0;
				}
			}
		}
	}
	inv_A = multiply(inv_U, inv_L);
	return inv_A;
}
//矩阵A*num=矩阵B，并返回
vector<vector<double>> multiply_num123(const vector<vector<double>>& A, double num)
{
    int A_h = A.size();
    int A_l = A[0].size();
    vector<vector<double>> B = creatmatrix(A_h, A_l);
    for (int i = 0; i < A_h; i++)
    {
        for (int j = 0; j < A_l; j++)
        {
            B[i][j] = num * A[i][j];
        }
    }
    return B;
}
//矩阵输出显示
void show_matrix(const vector<vector<double>>& A)
{
	int h = A.size();
	int l = A[0].size();
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < l; j++)
		{
			cout << A[i][j] << "\t";
		}
		cout << endl;
	}
	cout << "------------------" << endl;
}
//状态空间方程
void get_model_matrix(vector<vector<double>>* A, vector<vector<double>>* B) {
	vector<vector<double>> a, b,C;
    //关于质量的矩阵参数计算
    double m11 = M + 2 * m + 2 * I_wheel / R_wheel / R_wheel;
    double m12 =M*l_bar*cos(angle);
    double m22 = M * l_bar * l_bar+I_jiti_y;
    double m33 = I_jiti_z+2*I_wheel+(m+I_wheel/R_wheel / R_wheel)*D*D/2-(I_jiti_z-I_jiti_x-M*l_bar*l_bar)*sin(angle)*sin(angle)  ;
    double d12 = -M*l_bar* anglevelocity*sin(angle)-2*damping/R_wheel;
    double d13 = M * l_bar * yawvelocity * sin(angle);
    double d23 = (I_jiti_z - I_jiti_x - M * l_bar * l_bar) * yawvelocity * sin(angle) * cos(angle);
    double d31 = M * l_bar * yawvelocity * sin(angle);
    double d32 = -d23;
    double d33 = -(I_jiti_z - I_jiti_x - M * l_bar * l_bar) * anglevelocity * sin(angle) * cos(angle) + D * D * damping / 2 / R_wheel / R_wheel;
    vector<vector<double>> matrix_M = {
                    {m11,        m12,      0.0 },
                    {m12,        m22,      0.0},
                    {0.0,          0.0,    m33},
     };
    vector<vector<double>> matrix_A = {
                {2*damping/R_wheel/R_wheel,        d12,      d13 },
                {-2 * damping / R_wheel,        2 * damping,      d23},
                {d31,          d32,       d33},
    };
    vector<vector<double>> MA ;
    MA = multiply(matrix_M, matrix_A);    //三阶状态空间控制矩阵就是这个，接下来需要把它变化为四阶的。
    //其实是负的M*A,但是矩阵乘法还不能负数，再后面AAA转换时进行了处理
    vector<vector<double>> AAA = {
            {0,        0,      0,      0 },
            {0,        0,      0,      0 },
            {0,        0,      0,      0 },
            {0,        0,      0,      0 }
    };
       
    for (int i = 0; i++; i < 3) {
        for (int j = 0; j++; j < 3) {
            AAA[i][j] = -MA[i][j];   //负的MA
        }    
    }
   
    AAA[0][3] = -1* m12 * m33 * M * l_bar * g;
    AAA[1][3] = m11 * m33 * M * l_bar * g;
    AAA[2][3] = 0;
    AAA[3][0] = 0;
    AAA[3][1] = 1;
    AAA[3][2] = 0;
    AAA[3][3] = 0;


	a = { {0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, A23, 0.0},
		{0.0, 0.0, 0.0, 1.0},
		{0.0, 0.0, A43, 0.0} };
    C =   { {1.0, 0.0, 0.0, 0.0},  
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0}  };

    a = plusmatrix(multiply_num123(a, 0.0005),C);

	show_matrix(a);
	b = { {1/R_wheel  ,   1 / R_wheel },
		  {-1,   -1},
		  {D/2,   -D/2},
		  {0.0,   0.0}  
    };
   b = multiply_num123(b, 0.0005);
	show_matrix(b);
	*A = a;
	*B = b;
}
//求解里蒂卡方程
vector<vector<double>> solve_DARE(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R) {
	int maxiter = 150;   /*迭代次数*/
	double eps = 0.001;  //迭代误差要求
	double maxunit=0;
	vector<vector<double>> P1,P2, P3,Pplus, P4, P5, P6,P7, Pn, Pmins;
	vector<vector<double>> P = Q;          /*不确定这种用法有没有错*/
	for (int i = 0; i < maxiter; i++) {
		P1 = multiply_tri(trans(A), P, A);
		//show_matrix(P1);
		P2 = multiply_tri(trans(A), P, B);
		//show_matrix(P2);
		P3 = multiply_tri(trans(B), P, B);
		//show_matrix(P3);
		//show_matrix(R);
		Pplus = plusmatrix(R, P3);
		//show_matrix(Pplus);
	//	P4 = inverse(plusmatrix(R, P3));

        P4 = inverse(Pplus);

		//show_matrix(P4);
		P5 = multiply_tri(trans(B), P, A);
		//show_matrix(P5);
		P6 = multiply_tri(P2, P4, P5);
		//show_matrix(P6);
		P7= minusmatrix(P1, P6);
		//show_matrix(P7);
		Pn = plusmatrix(P7, Q);
		//show_matrix(Pn);
		
		Pmins = minusmatrix(Pn, P);
		//show_matrix(Pmins);
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				if (Pmins[j][k] > maxunit)
					maxunit = Pn[j][k];
			}
		}

		//cout << maxunit << endl;
		//求取矩阵最大单位，接下来设置满足迭代误差后跳出循环
		if (maxunit < eps)
		{
			break;
		}
		P = Pn;
		maxunit = 0;

	}
	cout << maxunit << endl;
	return Pn;
}
//求解里蒂卡方程
vector<vector<double>> dlqr(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R) {
	vector<vector<double>> P, P1, P2, P3, P4, P5;

	P = solve_DARE(A, B, Q, R);
	//show_matrix(P);
	P1 = multiply_tri(trans(B), P, B);
	//show_matrix(P1);
	P2 = plusmatrix(P1, R);
	//show_matrix(P2);
    vector<vector<double>> X = inverse(P2);
    
	P3 = multiply_tri(trans(B), P, A);
	//show_matrix(P3);

	P4 = multiply(X, P3);
	//show_matrix(P4);
	return P4;

}
//LQR控制方法
vector<vector<double>> lqr_control(const vector<vector<double>>& x) {
	vector<vector<double>> A, B, K, u;
	get_model_matrix(&A, &B);
	K = dlqr(A, B, Q, R);
    show_matrix(K);
	u = multiply(K, x);
	u[0][0] = u[0][0] * -1 * 2;

	return u;    //u应该就是力矩，webots里的power，两个轮子应该还要除二。
}





int main(int argc, char** argv) {
	ofstream torque_test;
	torque_test.open("torque.txt", ios::out);
	torque_test << "时间\t" << "电机力矩\t" << "位置\t" << "速度\t" << "倾角\t" << "倾角加速度" << endl;
	Robot* robot = new Robot();
	//定义步长
	int timeStep = (int)robot->getBasicTimeStep();

	//添加2，10 10 3 分别是大腿长度，小腿长度和轮半径
	WheeledBiped* robotkine = new WheeledBiped(l_datui, l_xiaotui, R_wheel);
	//添加2完毕

	//定义6个电机
	Motor* lefthip_motor = robot->getMotor("r_hip_motor");
	Motor* righthip_motor = robot->getMotor("l_hip_motor");
	Motor* leftknee_motor = robot->getMotor("l_knee_motor");
	Motor* rightknee_motor = robot->getMotor("r_knee_motor");
	Motor* leftwheel_motor = robot->getMotor("l_wheel_motor");
	Motor* rightwheel_motor = robot->getMotor("r_wheel_motor");
	//定义6个编码器
	PositionSensor* lefthip_sensor = robot->getPositionSensor("l_hip_poi_sensor");
	PositionSensor* leftknee_sensor = robot->getPositionSensor("l_knee_poi_sensor");
	PositionSensor* leftwheel_sensor = robot->getPositionSensor("l_wheel_poi_sensor");
	PositionSensor* righthip_sensor = robot->getPositionSensor("r_hip_poi_sensor");
	PositionSensor* rightknee_sensor = robot->getPositionSensor("r_knee_poi_sensor");
	PositionSensor* rightwheel_sensor = robot->getPositionSensor("r_wheel_poi_sensor");
	//定义imu
	InertialUnit* imu = robot->getInertialUnit("imu");
    //定义键盘
    Keyboard kb;
	//电机使能
	lefthip_motor->enableTorqueFeedback(timeStep);
	righthip_motor->enableTorqueFeedback(timeStep);
	leftknee_motor->enableTorqueFeedback(timeStep);
	rightknee_motor->enableTorqueFeedback(timeStep);
	leftwheel_motor->enableTorqueFeedback(timeStep);
	rightwheel_motor->enableTorqueFeedback(timeStep);

	//传感器使能
	lefthip_sensor->enable(timeStep);
	leftknee_sensor->enable(timeStep);
	leftwheel_sensor->enable(timeStep);
	righthip_sensor->enable(timeStep);
	rightknee_sensor->enable(timeStep);
	rightwheel_sensor->enable(timeStep);
	imu->enable(timeStep);
    //键盘使能
    kb.enable(timeStep);

	//添加3 添加记录上一个时刻的编码器位置，以获得关节角速度
	double prelefthip_position = 0;
	double preleftknee_position = 0;
	double preleftwheel_position = 0;
	double prerighthip_position = 0;
	double prerightknee_position = 0;
	double prerightwheel_position = 0;
	double preanglevelocity = 0; //身体倾斜速度
	double prepitch = 0;  //imu 俯仰角
    double preyawvelocity = 0;  //航向角速度
	//添加3结束



	// Main loop:
	// - perform simulation steps until Webots is stopping the controller

	while (robot->step(timeStep) != -1) {
		// Read the sensors:
		double lefthip_position_o = lefthip_sensor->getValue();
		double lefthip_position = lefthip_position_o - 1.308997;
		//显示是0但是此时夹角75度，可能方向有误
		double leftknee_position_o = leftknee_sensor->getValue();
		double leftknee_position = leftknee_position_o + 2.617994;
		//显示是0但是此时夹角150度，可能方向有误
		double leftwheel_position = leftwheel_sensor->getValue();
		//可能方向有误
 		double righthip_position_o = righthip_sensor->getValue();
		double righthip_position = righthip_position_o - 1.308997;
		//显示是0但是此时夹角75度，可能方向有误
		double rightknee_position_o = rightknee_sensor->getValue();
		double rightknee_position = rightknee_position_o + 2.617994;
		//显示是0但是此时夹角150度，可能方向有误
		double rightwheel_position = rightwheel_sensor->getValue();

		double roll = imu->getRollPitchYaw()[0];   //横滚角，初始为0
		double pitch_o = imu->getRollPitchYaw()[1];  
		//翻转角，初始为0.方向为后倒（腿在后）为负
		double pitch = pitch_o ;
		double yaw = imu->getRollPitchYaw()[2];    //航向角，初始为0

		cout << " 横滚角： " << roll / 3.1415923 * 180 << "度"
			<< " 翻转角： " << pitch / 3.1415923 * 180 << "度"
			<< " 航向角： " << yaw / 3.1415923 * 180 << "度" << endl;

		//添加4 计算各个电机角速度
		double lefthip_positiondot = (lefthip_position - prelefthip_position) / timeStep * 1000;
		double leftknee_positiondot = (leftknee_position - preleftknee_position)/timeStep * 1000;
		double leftwheel_positiondot = (leftwheel_position - preleftwheel_position)/timeStep * 1000;
		double righthip_positiondot = (righthip_position - prerighthip_position)/timeStep * 1000;
		double rightknee_positiondot = (rightknee_position - prerightknee_position)/timeStep * 1000;
		double rightwheel_positiondot = (rightwheel_position - prerightwheel_position)/timeStep * 1000;
		double pitchdot = (pitch - prepitch) / timeStep * 1000;
		//添加4完毕

	cout << "左髋部角度：" << lefthip_position << '\t';
	cout << "左膝盖角度：" << leftknee_position << '\t';
	cout << "左轮角度：" << leftwheel_position << '\n';
	cout << "右髋部角度：" << righthip_position << '\t';
	cout << "右膝盖角度：" << rightknee_position << '\t';
	cout << "右轮角度：" << rightwheel_position << '\n';

		//添加5 获取位置 速度 倾斜角 倾斜角速度 倾角角加速度,上面还没有imu,所以imu处先填写0,时间间隔取的0.1；
		
		double x = robotkine->getlocation(lefthip_position, leftknee_position, leftwheel_position, pitch);//获得在该时刻的位置
		double v = robotkine->getvelocity(lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, 
            righthip_positiondot, rightknee_positiondot, rightwheel_positiondot, pitch);//获得在该时刻的速度
		double angle = robotkine->getangle(lefthip_position, leftknee_position, leftwheel_position, pitch);//获得在该时刻的倾角
		double anglevelocity = robotkine->getanglevelocity(lefthip_position, leftknee_position, leftwheel_position, pitch,
			lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, pitchdot);//获得在该时刻的倾角速度!!!没有考虑左右轮不同时候的情况
        double yawvelocity = robotkine->getyawvelocity(lefthip_position, leftknee_position, leftwheel_position, 
            righthip_position, rightknee_position, rightwheel_position, pitch);//获得在该时刻的转向速度
		double angleacceleration = (anglevelocity - preanglevelocity) / timeStep;

		//添加6 更新前一个编码器位置
		prelefthip_position = lefthip_position;
		preleftknee_position = leftknee_position;
		preleftwheel_position = leftwheel_position;
		prerighthip_position = righthip_position;
		prerightknee_position = righthip_position;
		prerightwheel_position = rightwheel_position;
		preanglevelocity = anglevelocity;
		prepitch = pitch;
        preyawvelocity = yawvelocity;
		//添加5 6完毕
		//所有添加完毕，已获得x，v，angle，anglevelocity，angleacceleration可供使用
		//未考虑电机安装初始角问题，在webots确定初始位置后，在传感器读取时减去初始值。
			
		//系统的初始的状态，之后由传感器输入!!分别是位移 速度 倾斜角 倾斜角速度
		vector<vector<double>> x0 = { {v- v_target}, {anglevelocity - anglevle_target}, {- yaw_target}, {anglevelocity- anglevle_target} };
		cout << "轮胎位移：" << x << "m" << '\t';
		cout << "轮胎速度：" << v << "m/s" << '\t';
		cout << "机身倾斜角：" << angle/3.141593*180 << "度" << '\t';
		cout << "机身角速度：" << anglevelocity / 3.141593 * 180 << "rad/s" << '\n';
		



	//	torque_test << lefthip_sensor->getValue() << "\t" << leftknee_sensor->getValue() << "\t"  << lefthip_motor->getTorqueFeedback() << "\t" << leftknee_motor->getTorqueFeedback() << endl;

		vector<vector<double>> x1 = x0;

		vector<vector<double>> u;
		t = t + 0.001 * timeStep;
        int key=87;
        key = kb.getKey();
        cout << key << endl;
        double lefttorque = 0;
        double righttorque = 0;
   
        if (key == 87)
            {
            leftwheel_motor->setVelocity(0.3);
            rightwheel_motor->setVelocity(0.3);
                cout << "前进" << endl;
            }
        else if (key == 83)
        {
            leftwheel_motor->setVelocity(-0.3);
            rightwheel_motor->setVelocity(-0.3);
            cout << "后退" << endl;
        }
        else
        {
            u = lqr_control(x1);
            // Enter here functions to send actuator commands, like:
             lefttorque = u[0][0];
             righttorque = u[0][0];
             leftwheel_motor->setTorque(lefttorque);
             rightwheel_motor->setTorque(righttorque);
            //leftwheel_motor->setTorque(u[0][0]);
            //rightwheel_motor->setTorque(u[0][0]);
            cout << "12345" << endl;

        }



        

        cout << "电机力矩:" << lefttorque << endl;
        cout << "时间:" << t << endl;
        cout << "---------------------------------------------------------------------------" << endl;
        // Process sensor data here.
         	torque_test << t << "\t"
                    << lefttorque << "\t"
					<< x << "\t"
					<< v << "\t"
					<< angle-angle_target << "\t"
					<< anglevelocity << endl;
	}

	// Enter here exit cleanup code.
	torque_test.close();
	delete robot;
	return 0;
}

