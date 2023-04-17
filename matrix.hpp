// File:          matrix.hpp
// Date:          2023.4.11
// Description:   ����lqr�Ŀ��Ʒ���
// Author:        xuzhanhao
// Modifications: 
#ifndef Matrix_H_H
#define Matrix_H_H


#include <iostream>
#include <vector>
#include <math.h>
using namespace std;
double epsilon = 0.0001;
class matrix
{
public:
	~matrix();
	vector<vector<double>> creatmatrix(int h, int l);
	vector<vector<double>> multiply_num(const vector<vector<double>>& A, double num);
	vector<vector<double>> plusmatrix(const vector<vector<double>>& A, const vector<vector<double>>& B);
	vector<vector<double>> multiply(const vector<vector<double>>& A, const vector<vector<double>>& B);
	vector<vector<double>> minusmatrix(const vector<vector<double>>& A, const vector<vector<double>>& B);
	vector<vector<double>> multiply_tri(const vector<vector<double>>& A, const vector<vector<double>>& B,
		const vector<vector<double>>& C);
	vector<vector<double>> trans(const vector<vector<double>>& A);
	vector<vector<double>> inverse(const vector<vector<double>>& A);
	void show_matrix(const vector<vector<double>>& A);
	/* theta1 2 3�ֱ����Źؽڣ�ϥ�ؽڣ��ֵ���ĽǶȡ�theta1 2 3dot�ֱ������ǵĽ��ٶȡ�IMU�ĽǶȺͽ��ٶ�����XOYƽ���� */
};

matrix::~matrix()
{
}
//��������ĺ���
vector<vector<double>> matrix::creatmatrix(int h, int l)
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
vector<vector<double>> matrix::multiply_num(const vector<vector<double>>& A, double num)
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
vector<vector<double>> matrix::plusmatrix(const vector<vector<double>>& A, const vector<vector<double>>& B)
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
vector<vector<double>> matrix::multiply(const vector<vector<double>>& A, const vector<vector<double>>& B)
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
vector<vector<double>> matrix::minusmatrix(const vector<vector<double>>& A, const vector<vector<double>>& B)
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
vector<vector<double>> matrix::multiply_tri(const vector<vector<double>>& A, const vector<vector<double>>& B,
	const vector<vector<double>>& C)
{
	vector<vector<double>> D, E;
	D = multiply(A, B);
	E = multiply(D, C);
	return E;
}
//�������A���������A��ת�þ���AT
vector<vector<double>> matrix::trans(const vector<vector<double>>& A)
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
vector<vector<double>> matrix::inverse(const vector<vector<double>>& A)
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
//���������ʾ
void matrix::show_matrix(const vector<vector<double>>& A)
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

#endif // !Matrix_H_H