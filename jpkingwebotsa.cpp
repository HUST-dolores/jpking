// File:          jpking_lqr.cpp
// Date:          2022.10.17
// Description:   ����lqr�Ŀ��Ʒ���
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
// ���1���

using namespace webots;
using namespace std;
double theta2 = 0.5236;//����С�ȼнǣ�ĿǰΪ30��
double l_bar = 0.3*sin(theta2/2)*2;  // �����ȳ�
double M = 4.97;  // ���������Ч����[kg]
double m = 0.15708; // ��������[kg]
double g = 9.8; // [m / s ^ 2]
double l_datui = 0.3;   //���ȳ���[m]
double l_xiaotui = 0.3;  //С�ȳ���[m]
double R_wheel = 0.05;// ���Ӱ뾶[m]
double D = 0.30 + 2 * 0.015 + 2 * 0.03 + 0.02; //����֮��ľ���
double damping = 0.01;
double I_wheel = m * R_wheel * R_wheel / 2;
double I_jiti_x = M * (0.26 * 0.26 + 0.13 * 0.13) / 12;
double I_jiti_z = M * (0.26 * 0.26 + 0.13 * 0.13) / 12;
double I_jiti_y = M * (0.13 * 0.13 + 0.13 * 0.13) / 12;//!!!�������Ȳ�������
double t = 0.0; //ʱ��
const double epsilon = 0.0001;

double angle,anglevelocity, yawvelocity;//��б��
double m11, m12, m22, m33;//������������
//��ض���ѧ��������
double a_dyna = R_wheel * (M + 2 * m + 2 * I_wheel / R_wheel / R_wheel);
double b_dyna = M * R_wheel * l_bar;
double c_dyna = I_jiti_z + M * l_bar * l_bar;
double d_dyna = M * g * l_bar;
double e_dyna = M * l_bar;
double f_dyna = 1 / (R_wheel * (m * D + I_wheel * D / R_wheel / R_wheel + 2 * I_jiti_y / D));
//״̬�ռ䷽�̲�������
double A23 = (0 - b_dyna * d_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double A43 = (    a_dyna * d_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double B21 = (c_dyna + b_dyna )    / (a_dyna * c_dyna - b_dyna * e_dyna);
double B41 = (0 - e_dyna-a_dyna)   / (a_dyna * c_dyna - b_dyna * e_dyna);
double B61 = f_dyna;
double B62 = -f_dyna;

//����Q\R
vector<vector<double>> Q = {
				{100.0,  0.0,      0.0,    0.0},
				{0.0,  1.0,    0.0,    0.0},
				{0.0,  0.0,      1.0,    0.0},
				{0.0,  0.0,      0.0,    10.0}
};
vector<vector<double>> R = { {10,0},{0,10} };
//����Ŀ��Ĳ���
double x_target = 3;
double v_target = 0;
double angle_target = 0.19185;  //���ֵӦ�������Ż��������ĵ�λ�ö��ı��
double anglevle_target = 0;
double yaw_target = 0;




//��������ĺ���
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
//����˳���������
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
//����A+����B=����C��������
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
//����A*����B=����C��������
vector<vector<double>> multiply(const vector<vector<double>>& A, const vector<vector<double>>& B)
{
	int A_h = A.size();
	int A_l = A[0].size();
	int B_h = B.size();
	int B_l = B[0].size();
	if (A_l != B_h)
	{
		cout << "������ά���޷����" << endl;
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
//����A-����B=����C��������
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
//����A*����B*����C=Ŀ����󣬲�����
vector<vector<double>> multiply_tri(const vector<vector<double>>& A, const vector<vector<double>>& B,
const vector<vector<double>>& C)
{
	vector<vector<double>> D, E;
	D = multiply(A, B);
	E = multiply(D, C);
	return E;
}
//�������A���������A��ת�þ���AT
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
//�������A,�������A�������inv_A
vector<vector<double>> inverse(const vector<vector<double>>& A)
{
	if (A.size() != A[0].size())
	{
		cout << "�������ά�����Ϸ�" << endl;
		exit(0);
	}
	int n = A.size();
	vector<vector<double>> inv_A = creatmatrix(n, n);
	vector<vector<double>> L = creatmatrix(n, n);
	vector<vector<double>> U = creatmatrix(n, n);
	vector<vector<double>> inv_L = creatmatrix(n, n);
	vector<vector<double>> inv_U = creatmatrix(n, n);
	//LU�ֽ�
		//L����Խ�Ԫ��Ϊ1
	for (int i = 0; i < n; i++)
	{
		L[i][i] = 1;
	}
	//U�����һ��
	for (int i = 0; i < n; i++)
	{
		U[0][i] = A[0][i];
	}
	//L�����һ��
	for (int i = 1; i < n; i++)
	{
		L[i][0] = 1.0 * A[i][0] / A[0][0];
	}

	//����LU��������
	for (int i = 1; i < n; i++)
	{
		//����U��i��j�У�
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
		//����L��j��i�У�
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
	//L Uʣ��λ����Ϊ0
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
	//LU����
	//�����U���� 
	for (int i = 0; i < n; i++)
	{
		inv_U[i][i] = 1 / U[i][i];// U�Խ�Ԫ�ص�ֵ��ֱ��ȡ����
		for (int k = i - 1; k >= 0; k--)
		{
			double s = 0;
			for (int j = k + 1; j <= i; j++)
			{
				s = s + U[k][j] * inv_U[j][i];
			}
			inv_U[k][i] = -s / U[k][k];//�������㣬���е������εõ�ÿһ��ֵ��
			if (abs(inv_U[k][i]) < epsilon)
			{
				inv_U[k][i] = 0.0;
			}
		}
	}
	//�����L����
	for (int i = 0; i < n; i++)
	{
		inv_L[i][i] = 1; //L�Խ�Ԫ�ص�ֵ��ֱ��ȡ����������Ϊ1
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
//����A*num=����B��������
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
//���������ʾ
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
//״̬�ռ䷽��
void get_model_matrix(vector<vector<double>>* A, vector<vector<double>>* B) {
	vector<vector<double>> a, b,C;
    //���������ľ����������
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
    MA = multiply(matrix_M, matrix_A);    //����״̬�ռ���ƾ�������������������Ҫ�����仯Ϊ�Ľ׵ġ�
    //��ʵ�Ǹ���M*A,���Ǿ���˷������ܸ������ٺ���AAAת��ʱ�����˴���
    vector<vector<double>> AAA = {
            {0,        0,      0,      0 },
            {0,        0,      0,      0 },
            {0,        0,      0,      0 },
            {0,        0,      0,      0 }
    };
       
    for (int i = 0; i++; i < 3) {
        for (int j = 0; j++; j < 3) {
            AAA[i][j] = -MA[i][j];   //����MA
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
//�����ٿ�����
vector<vector<double>> solve_DARE(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R) {
	int maxiter = 150;   /*��������*/
	double eps = 0.001;  //�������Ҫ��
	double maxunit=0;
	vector<vector<double>> P1,P2, P3,Pplus, P4, P5, P6,P7, Pn, Pmins;
	vector<vector<double>> P = Q;          /*��ȷ�������÷���û�д�*/
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
		//��ȡ�������λ���������������������������ѭ��
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
//�����ٿ�����
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
//LQR���Ʒ���
vector<vector<double>> lqr_control(const vector<vector<double>>& x) {
	vector<vector<double>> A, B, K, u;
	get_model_matrix(&A, &B);
	K = dlqr(A, B, Q, R);
    show_matrix(K);
	u = multiply(K, x);
	u[0][0] = u[0][0] * -1 * 2;

	return u;    //uӦ�þ������أ�webots���power����������Ӧ�û�Ҫ������
}





int main(int argc, char** argv) {
	ofstream torque_test;
	torque_test.open("torque.txt", ios::out);
	torque_test << "ʱ��\t" << "�������\t" << "λ��\t" << "�ٶ�\t" << "���\t" << "��Ǽ��ٶ�" << endl;
	Robot* robot = new Robot();
	//���岽��
	int timeStep = (int)robot->getBasicTimeStep();

	//���2��10 10 3 �ֱ��Ǵ��ȳ��ȣ�С�ȳ��Ⱥ��ְ뾶
	WheeledBiped* robotkine = new WheeledBiped(l_datui, l_xiaotui, R_wheel);
	//���2���

	//����6�����
	Motor* lefthip_motor = robot->getMotor("r_hip_motor");
	Motor* righthip_motor = robot->getMotor("l_hip_motor");
	Motor* leftknee_motor = robot->getMotor("l_knee_motor");
	Motor* rightknee_motor = robot->getMotor("r_knee_motor");
	Motor* leftwheel_motor = robot->getMotor("l_wheel_motor");
	Motor* rightwheel_motor = robot->getMotor("r_wheel_motor");
	//����6��������
	PositionSensor* lefthip_sensor = robot->getPositionSensor("l_hip_poi_sensor");
	PositionSensor* leftknee_sensor = robot->getPositionSensor("l_knee_poi_sensor");
	PositionSensor* leftwheel_sensor = robot->getPositionSensor("l_wheel_poi_sensor");
	PositionSensor* righthip_sensor = robot->getPositionSensor("r_hip_poi_sensor");
	PositionSensor* rightknee_sensor = robot->getPositionSensor("r_knee_poi_sensor");
	PositionSensor* rightwheel_sensor = robot->getPositionSensor("r_wheel_poi_sensor");
	//����imu
	InertialUnit* imu = robot->getInertialUnit("imu");
    //�������
    Keyboard kb;
	//���ʹ��
	lefthip_motor->enableTorqueFeedback(timeStep);
	righthip_motor->enableTorqueFeedback(timeStep);
	leftknee_motor->enableTorqueFeedback(timeStep);
	rightknee_motor->enableTorqueFeedback(timeStep);
	leftwheel_motor->enableTorqueFeedback(timeStep);
	rightwheel_motor->enableTorqueFeedback(timeStep);

	//������ʹ��
	lefthip_sensor->enable(timeStep);
	leftknee_sensor->enable(timeStep);
	leftwheel_sensor->enable(timeStep);
	righthip_sensor->enable(timeStep);
	rightknee_sensor->enable(timeStep);
	rightwheel_sensor->enable(timeStep);
	imu->enable(timeStep);
    //����ʹ��
    kb.enable(timeStep);

	//���3 ��Ӽ�¼��һ��ʱ�̵ı�����λ�ã��Ի�ùؽڽ��ٶ�
	double prelefthip_position = 0;
	double preleftknee_position = 0;
	double preleftwheel_position = 0;
	double prerighthip_position = 0;
	double prerightknee_position = 0;
	double prerightwheel_position = 0;
	double preanglevelocity = 0; //������б�ٶ�
	double prepitch = 0;  //imu ������
    double preyawvelocity = 0;  //������ٶ�
	//���3����



	// Main loop:
	// - perform simulation steps until Webots is stopping the controller

	while (robot->step(timeStep) != -1) {
		// Read the sensors:
		double lefthip_position_o = lefthip_sensor->getValue();
		double lefthip_position = lefthip_position_o - 1.308997;
		//��ʾ��0���Ǵ�ʱ�н�75�ȣ����ܷ�������
		double leftknee_position_o = leftknee_sensor->getValue();
		double leftknee_position = leftknee_position_o + 2.617994;
		//��ʾ��0���Ǵ�ʱ�н�150�ȣ����ܷ�������
		double leftwheel_position = leftwheel_sensor->getValue();
		//���ܷ�������
 		double righthip_position_o = righthip_sensor->getValue();
		double righthip_position = righthip_position_o - 1.308997;
		//��ʾ��0���Ǵ�ʱ�н�75�ȣ����ܷ�������
		double rightknee_position_o = rightknee_sensor->getValue();
		double rightknee_position = rightknee_position_o + 2.617994;
		//��ʾ��0���Ǵ�ʱ�н�150�ȣ����ܷ�������
		double rightwheel_position = rightwheel_sensor->getValue();

		double roll = imu->getRollPitchYaw()[0];   //����ǣ���ʼΪ0
		double pitch_o = imu->getRollPitchYaw()[1];  
		//��ת�ǣ���ʼΪ0.����Ϊ�󵹣����ں�Ϊ��
		double pitch = pitch_o ;
		double yaw = imu->getRollPitchYaw()[2];    //����ǣ���ʼΪ0

		cout << " ����ǣ� " << roll / 3.1415923 * 180 << "��"
			<< " ��ת�ǣ� " << pitch / 3.1415923 * 180 << "��"
			<< " ����ǣ� " << yaw / 3.1415923 * 180 << "��" << endl;

		//���4 �������������ٶ�
		double lefthip_positiondot = (lefthip_position - prelefthip_position) / timeStep * 1000;
		double leftknee_positiondot = (leftknee_position - preleftknee_position)/timeStep * 1000;
		double leftwheel_positiondot = (leftwheel_position - preleftwheel_position)/timeStep * 1000;
		double righthip_positiondot = (righthip_position - prerighthip_position)/timeStep * 1000;
		double rightknee_positiondot = (rightknee_position - prerightknee_position)/timeStep * 1000;
		double rightwheel_positiondot = (rightwheel_position - prerightwheel_position)/timeStep * 1000;
		double pitchdot = (pitch - prepitch) / timeStep * 1000;
		//���4���

	cout << "���Ų��Ƕȣ�" << lefthip_position << '\t';
	cout << "��ϥ�ǽǶȣ�" << leftknee_position << '\t';
	cout << "���ֽǶȣ�" << leftwheel_position << '\n';
	cout << "���Ų��Ƕȣ�" << righthip_position << '\t';
	cout << "��ϥ�ǽǶȣ�" << rightknee_position << '\t';
	cout << "���ֽǶȣ�" << rightwheel_position << '\n';

		//���5 ��ȡλ�� �ٶ� ��б�� ��б���ٶ� ��ǽǼ��ٶ�,���滹û��imu,����imu������д0,ʱ����ȡ��0.1��
		
		double x = robotkine->getlocation(lefthip_position, leftknee_position, leftwheel_position, pitch);//����ڸ�ʱ�̵�λ��
		double v = robotkine->getvelocity(lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, 
            righthip_positiondot, rightknee_positiondot, rightwheel_positiondot, pitch);//����ڸ�ʱ�̵��ٶ�
		double angle = robotkine->getangle(lefthip_position, leftknee_position, leftwheel_position, pitch);//����ڸ�ʱ�̵����
		double anglevelocity = robotkine->getanglevelocity(lefthip_position, leftknee_position, leftwheel_position, pitch,
			lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, pitchdot);//����ڸ�ʱ�̵�����ٶ�!!!û�п��������ֲ�ͬʱ������
        double yawvelocity = robotkine->getyawvelocity(lefthip_position, leftknee_position, leftwheel_position, 
            righthip_position, rightknee_position, rightwheel_position, pitch);//����ڸ�ʱ�̵�ת���ٶ�
		double angleacceleration = (anglevelocity - preanglevelocity) / timeStep;

		//���6 ����ǰһ��������λ��
		prelefthip_position = lefthip_position;
		preleftknee_position = leftknee_position;
		preleftwheel_position = leftwheel_position;
		prerighthip_position = righthip_position;
		prerightknee_position = righthip_position;
		prerightwheel_position = rightwheel_position;
		preanglevelocity = anglevelocity;
		prepitch = pitch;
        preyawvelocity = yawvelocity;
		//���5 6���
		//���������ϣ��ѻ��x��v��angle��anglevelocity��angleacceleration�ɹ�ʹ��
		//δ���ǵ����װ��ʼ�����⣬��webotsȷ����ʼλ�ú��ڴ�������ȡʱ��ȥ��ʼֵ��
			
		//ϵͳ�ĳ�ʼ��״̬��֮���ɴ���������!!�ֱ���λ�� �ٶ� ��б�� ��б���ٶ�
		vector<vector<double>> x0 = { {v- v_target}, {anglevelocity - anglevle_target}, {- yaw_target}, {anglevelocity- anglevle_target} };
		cout << "��̥λ�ƣ�" << x << "m" << '\t';
		cout << "��̥�ٶȣ�" << v << "m/s" << '\t';
		cout << "������б�ǣ�" << angle/3.141593*180 << "��" << '\t';
		cout << "������ٶȣ�" << anglevelocity / 3.141593 * 180 << "rad/s" << '\n';
		



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
                cout << "ǰ��" << endl;
            }
        else if (key == 83)
        {
            leftwheel_motor->setVelocity(-0.3);
            rightwheel_motor->setVelocity(-0.3);
            cout << "����" << endl;
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



        

        cout << "�������:" << lefttorque << endl;
        cout << "ʱ��:" << t << endl;
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

