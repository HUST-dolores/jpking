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
	torque_test << "电机力矩\t" << "x\t" << "v\t" << "倾角\t" << "倾角加速度" << endl;

	//定义步长
	int timeStep = (int)robot->getBasicTimeStep();

	//添加2，10 10 3 分别是大腿长度，小腿长度和轮半径

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


	//添加3 添加记录上一个时刻的编码器位置，以获得关节角速度
	double prelefthip_position = 0;
	double preleftknee_position = 0;
	double preleftwheel_position = 0;
	double prerighthip_position = 0;
	double prerightknee_position = 0;
	double prerightwheel_position = 0;
	double preanglevelocity = 0; //身体倾斜速度
	double prepitch = 0;  //imu 俯仰角
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
		double v = robotkine->getvelocity(lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, pitch);//获得在该时刻的速度
		double angle = robotkine->getangle(lefthip_position, leftknee_position, leftwheel_position, pitch);//获得在该时刻的倾角
		double anglevelocity = robotkine->getanglevelocity(lefthip_position, leftknee_position, leftwheel_position, pitch,
			lefthip_positiondot, leftknee_positiondot, leftwheel_positiondot, pitchdot);//获得在该时刻的倾角速度
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
		//添加5 6完毕
		//所有添加完毕，已获得x，v，angle，anglevelocity，angleacceleration可供使用
		//未考虑电机安装初始角问题，在webots确定初始位置后，在传感器读取时减去初始值。
			
		//系统的初始的状态，之后由传感器输入!!分别是位移 速度 倾斜角 倾斜角速度
		vector<vector<double>> x0 = { {x-x_target}, {v- v_target}, {angle-angle_target}, {anglevelocity-angle_target} };
		
		
		MiniPID pid_v = MiniPID(1, 0, 0);      //对速度进行pid控制
		MiniPID pid_h = MiniPID(1, 0, 0);               //对高度进行pid控制
		MiniPID pid_angle = MiniPID(1, 0, 0);           //对俯仰角进行pid控制

		while (true) {
			double output_v = pid_v.getOutput(x, x_target);
			double output_h = pid_h.getOutput(x, x_target);
			double output_angle = pid_angle.getOutput(x, x_target);
		}

		//此处添加一个函数，它的输入是pid的系数，还有误差，得到的是对应的




		cout << "轮胎位移：" << x << "m" << '\t';
		cout << "轮胎速度：" << v << "m/s" << '\t';
		cout << "机身倾斜角：" << angle/3.141593*180 << "度" << '\t';
		cout << "机身角速度：" << anglevelocity / 3.141593 * 180 << "rad/s" << '\n';
		



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

