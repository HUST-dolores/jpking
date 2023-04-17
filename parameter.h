#pragma once
double l_bar = 0.1553;  // �����ȳ�
double M = 4.97;  // ���������Ч����[kg]
double m = 0.15708; // ��������[kg]
double g = 9.8; // [m / s ^ 2]
double l_datui = 0.3;   //���ȳ���[m]
double l_xiaotui = 0.3;  //С�ȳ���[m]
double R_wheel = 0.05;// ���Ӱ뾶[m]
double D = 0.26 + 2 * 0.03 + 2 * 0.03 + 0.02; //����֮��ľ���
double I_wheel = m * R_wheel * R_wheel / 2;
double I_jiti_z = M * (0.26 * 0.26 + 0.13 * 0.13) / 12;
double I_jiti_y = M * (0.13 * 0.13 + 0.13 * 0.13) / 12;
double t = 0.0; //ʱ��
//��ض���ѧ��������
double a_dyna = R_wheel * (M + 2 * m + 2 * I_wheel / R_wheel / R_wheel);
double b_dyna = M * R_wheel * l_bar;
double c_dyna = I_jiti_z + M * l_bar * l_bar;
double d_dyna = M * g * l_bar;
double e_dyna = M * l_bar;
double f_dyna = 1 / (R_wheel * (m * D + I_wheel * D / R_wheel / R_wheel + 2 * I_jiti_y / D));
//״̬�ռ䷽�̲�������
double A23 = (0 - b_dyna * d_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double A43 = (a_dyna * d_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double B21 = (c_dyna + b_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double B41 = (0 - e_dyna - a_dyna) / (a_dyna * c_dyna - b_dyna * e_dyna);
double B61 = f_dyna;
double B62 = -f_dyna;
//����Q\R
vector<vector<double>> Q = {
				{10000.0,  0.0,      0.0,    0.0},
				{0.0,  10.0,    0.0,    0.0},
				{0.0,  0.0,      10.0,    0.0},
				{0.0,  0.0,      0.0,    10000.0}
};
vector<vector<double>> R = { {1000} };
//����Ŀ��Ĳ���
double x_target = 0;
double v_target = 1;
double angle_target = 0.19185;  //���ֵӦ�������Ż��������ĵ�λ�ö��ı��
double anglevle_target = 0;