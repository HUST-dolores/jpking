#pragma once
#include "matrix.hpp"
#include <iostream>
#include <math.h>


using namespace std;
matrix* Matrixlqr = new matrix();
class lqr
{
public:
	~lqr();
	void get_model_matrix(vector<vector<double>>* A, vector<vector<double>>* B);
	vector<vector<double>> solve_DARE(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R);
	vector<vector<double>> dlqr(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R);
	vector<vector<double>> lqr_control(const vector<vector<double>>& x);

	/* theta1 2 3�ֱ����Źؽڣ�ϥ�ؽڣ��ֵ���ĽǶȡ�theta1 2 3dot�ֱ������ǵĽ��ٶȡ�IMU�ĽǶȺͽ��ٶ�����XOYƽ���� */
};


lqr::~lqr()
{
}
//״̬�ռ䷽��
void lqr::get_model_matrix(vector<vector<double>>* A, vector<vector<double>>* B) {
	vector<vector<double>> a, b;
	a = { {0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, A23, 0.0},
		{0.0, 0.0, 0.0, 1.0},
		{0.0, 0.0, A43, 0,0} };
	//show_matrix(a);
	b = { {0.0},
		  {B21},
		  {0.0},
		  {B41} };
	//show_matrix(b);
	*A = a;
	*B = b;
}
//�����ٿ�����
vector<vector<double>> lqr::solve_DARE(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R) {
	int maxiter = 150;   /*��������*/
	double eps = 0.001;  //�������Ҫ��
	double maxunit = 0;
	vector<vector<double>> P1, P2, P3, Pplus, P4, P5, P6, P7, Pn, Pmins;
	vector<vector<double>> P = Q;          /*��ȷ�������÷���û�д�*/
	for (int i = 0; i < maxiter; i++) {
		P1 = Matrixlqr->multiply_tri(Matrixlqr->trans(A), P, A);
		//show_matrix(P1);
		P2 = Matrixlqr->multiply_tri(Matrixlqr->trans(A), P, B);
		//show_matrix(P2);
		P3 = Matrixlqr->multiply_tri(Matrixlqr->trans(B), P, B);
		//show_matrix(P3);
		//show_matrix(R);
		Pplus = Matrixlqr->plusmatrix(R, P3);
		//show_matrix(Pplus);
	//	P4 = inverse(plusmatrix(R, P3));
		P4 = { {1 / Pplus[0][0]} };
		//show_matrix(P4);
		P5 = Matrixlqr->multiply_tri(Matrixlqr->trans(B), P, A);
		//show_matrix(P5);
		P6 = Matrixlqr->multiply_tri(P2, P4, P5);
		//show_matrix(P6);
		P7 = Matrixlqr->minusmatrix(P1, P6);
		//show_matrix(P7);
		Pn = Matrixlqr->plusmatrix(P7, Q);
		//show_matrix(Pn);

		Pmins = Matrixlqr->minusmatrix(Pn, P);
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
vector<vector<double>> lqr::dlqr(const vector<vector<double>>& A, const vector<vector<double>>& B, const vector<vector<double>>& Q, const vector<vector<double>>& R) {
	vector<vector<double>> P, P1, P2, P3, P4, P5;
	vector<vector<double>> Pinver = { {0.0} };
	P = solve_DARE(A, B, Q, R);
	//show_matrix(P);
	P1 = Matrixlqr->multiply_tri(Matrixlqr->trans(B), P, B);
	//show_matrix(P1);
	P2 = Matrixlqr->plusmatrix(P1, R);
	//show_matrix(P2);
	double X = 1 / P2[0][0];
	P3 = Matrixlqr->multiply_tri(Matrixlqr->trans(B), P, A);
	//show_matrix(P3);
	Pinver[0][0] = X;
	P4 = Matrixlqr->multiply(Pinver, P3);
	//show_matrix(P4);
	return P4;

}
//LQR���Ʒ���
vector<vector<double>> lqr::lqr_control(const vector<vector<double>>& x) {
	vector<vector<double>> A, B, K, u;
	get_model_matrix(&A, &B);
	K = dlqr(A, B, Q, R);
	//K = { {-0.1000,-1.0491,-5.7867,-0.6230}};
	Matrixlqr->show_matrix(K);
	u = Matrixlqr->multiply(K, x);
	u[0][0] = u[0][0] * -1 * 2;
	cout << "�������:" << u[0][0] << endl;
	cout << "ʱ��:" << t << endl;
	cout << "---------------------------------------------------------------------------" << endl;
	return u;    //uӦ�þ������أ�webots���power����������Ӧ�û�Ҫ������
}
