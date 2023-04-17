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
#include <fstream>


using namespace webots;
using namespace std;

#include "WheeledBiped.hpp"
#include "matrix.hpp"
#include "parameter.h"
#include "lqr.hpp"
#include "MiniPID.h"
#include "MiniPID.cpp"

lqr* LQR = new lqr();
Robot* robot = new Robot();
WheeledBiped* robotkine = new WheeledBiped(l_datui, l_xiaotui, R_wheel);



int main(int argc, char** argv) {
	ofstream torque_test;
	torque_test.open("torque.txt", ios::out);
	torque_test << "�������\t" << "x\t" << "v\t" << "���\t" << "��Ǽ��ٶ�" << endl;

	//���岽��
	int timeStep = (int)robot->getBasicTimeStep();

	//���2��10 10 3 �ֱ��Ǵ��ȳ��ȣ�С�ȳ��Ⱥ��ְ뾶

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


	//���3 ��Ӽ�¼��һ��ʱ�̵ı�����λ�ã��Ի�ùؽڽ��ٶ�
	double prelefthip_position = 0;
	double preleftknee_position = 0;
	double preleftwheel_position = 0;
	double prerighthip_position = 0;
	double prerightknee_position = 0;
	double prerightwheel_position = 0;
	double preanglevelocity = 0; //������б�ٶ�
	double prepitch = 0;  //imu ������
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
		double v = robotkine->getvelocity(lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, pitch);//����ڸ�ʱ�̵��ٶ�
		double angle = robotkine->getangle(lefthip_position, leftknee_position, leftwheel_position, pitch);//����ڸ�ʱ�̵����
		double anglevelocity = robotkine->getanglevelocity(lefthip_position, leftknee_position, leftwheel_position, pitch,
			lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, pitchdot);//����ڸ�ʱ�̵�����ٶ�
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
		//���5 6���
		//���������ϣ��ѻ��x��v��angle��anglevelocity��angleacceleration�ɹ�ʹ��
		//δ���ǵ����װ��ʼ�����⣬��webotsȷ����ʼλ�ú��ڴ�������ȡʱ��ȥ��ʼֵ��
			
		//ϵͳ�ĳ�ʼ��״̬��֮���ɴ���������!!�ֱ���λ�� �ٶ� ��б�� ��б���ٶ�
		vector<vector<double>> x0 = { {x-x_target}, {v- v_target}, {angle-angle_target}, {anglevelocity-angle_target} };
		
		
		MiniPID pid_v = MiniPID(1, 0, 0);      //���ٶȽ���pid����
		MiniPID pid_h = MiniPID(1, 0, 0);               //�Ը߶Ƚ���pid����
		MiniPID pid_angle = MiniPID(1, 0, 0);           //�Ը����ǽ���pid����

		while (true) {
			double output_v = pid_v.getOutput(x, x_target);
			double output_h = pid_h.getOutput(x, x_target);
			double output_angle = pid_angle.getOutput(x, x_target);
		}

		//�˴����һ������������������pid��ϵ�����������õ����Ƕ�Ӧ��




		cout << "��̥λ�ƣ�" << x << "m" << '\t';
		cout << "��̥�ٶȣ�" << v << "m/s" << '\t';
		cout << "������б�ǣ�" << angle/3.141593*180 << "��" << '\t';
		cout << "������ٶȣ�" << anglevelocity / 3.141593 * 180 << "rad/s" << '\n';
		



	//	torque_test << lefthip_sensor->getValue() << "\t" << leftknee_sensor->getValue() << "\t"  << lefthip_motor->getTorqueFeedback() << "\t" << leftknee_motor->getTorqueFeedback() << endl;

		vector<vector<double>> x1 = x0;

		vector<vector<double>> u;
		t = t + 0.001 * timeStep;
		u = LQR->lqr_control(x1);

		// Process sensor data here.

		// Enter here functions to send actuator commands, like:
		leftwheel_motor->setTorque(u[0][0]);
		rightwheel_motor->setTorque(u[0][0]);
		//  motor->setPosition(10.0);
		torque_test << u[0][0] << "\t"
					<< x << "\t"
					<< v << "\t"
					<< angle-angle_target << "\t"
					<< anglevelocity << endl;
	};

	// Enter here exit cleanup code.
	torque_test.close();
	delete robot;
	return 0;
}

