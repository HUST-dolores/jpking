#pragma once
#include <iostream>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Brake.hpp>
#include <webots/InertialUnit.hpp>
#include <fstream>
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