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