#include <iostream>
#include <math.h>

class WheeledBiped
{
private:
    double l1; //第一段杆长，大腿长度
    double l2; //第二段杆长，小腿长度
    double R; //轮半径

public:
    WheeledBiped(double l1_in, double l2_in, double R_in);
    ~WheeledBiped();
    double getlocation(double theta1, double theta2, double theta3, double IMUtheta);
    double getvelocity(double theta1dot, double theta2dot, double theta3dot, double IMUthetadot);
    double getangle(double theta1, double theta2, double theta3, double IMUtheta);
    double getanglevelocity(double theta1, double theta2, double theta3, double IMUtheta, 
                            double theta1dot, double theta2dot, double theta3dot, double IMUthetadot);
    double getaltitude(double )
/* theta1 2 3分别是髋关节，膝关节，轮电机的角度。theta1 2 3dot分别是他们的角速度。IMU的角度和角速度是在XOY平面中 */
};

WheeledBiped::WheeledBiped(double l1_in,double l2_in,double R_in)
{
    l1 = l1_in;
    l2 = l2_in;
    R = R_in;
}

WheeledBiped::~WheeledBiped()
{
}

double WheeledBiped::getlocation(double theta1, double theta2, double theta3, double IMUtheta)
{
    return (theta3 + theta2 + theta1 + IMUtheta) * WheeledBiped::R;
}


//应该是轮子的位置
//double WheeledBiped::getlocation(double theta1, double theta2, double theta3, double //IMUtheta)
//{
//    return (theta3) * WheeledBiped::R;
//}

double WheeledBiped::getvelocity(double theta1dot, double theta2dot, double theta3dot, double IMUthetadot)
{
    return (theta3dot + theta2dot + theta1dot + IMUthetadot)*WheeledBiped::R;
}
 
//应该是轮子的速度
//double WheeledBiped::getvelocity(double theta1dot, double theta2dot, double theta3dot, double IMUthetadot)
//{
//    return (theta3dot) * WheeledBiped::R;
//}

double WheeledBiped::getangle(double theta1, double theta2, double theta3, double IMUtheta)
{
    double x = WheeledBiped::l1*sin(theta1) + WheeledBiped::l2*sin(theta1 + theta2);
    double y = WheeledBiped::l1*cos(theta1) + WheeledBiped::l2*cos(theta1 + theta2);
  //  double angle;
 //   if (atan2(x, -y) / 3.141593 * 180 > 90)
 //   {
 //       angle = atan2(x, -y) - 0.7853981;   //使angle在90度到-90度之间
  //  }
 //   if (atan2(x, -y) / 3.141593 * 180 < -90)
 //   {
 //       angle = atan2(x, -y) + 0.7853981;
  //  }
    std::cout << "jiaodu:" << atan2(x, y) + IMUtheta << std::endl;
    return atan2(x, y) + IMUtheta;
    //return IMUtheta;
}

double WheeledBiped::getanglevelocity(double theta1, double theta2, double theta3, double IMUtheta, 
                        double theta1dot, double theta2dot, double theta3dot, double IMUthetadot)
{
    double x = WheeledBiped::l1*sin(theta1) + WheeledBiped::l2*sin(theta1 + theta2);
    double y = WheeledBiped::l1*cos(theta1) + WheeledBiped::l2*cos(theta1 + theta2);
    double xdot = WheeledBiped::l1*cos(theta1)*theta1dot + 
                    WheeledBiped::l2*cos(theta1 + theta2)*(theta1dot + theta2dot);
    double ydot = - WheeledBiped::l1*sin(theta1)*theta1dot - 
                    WheeledBiped::l2*sin(theta1 + theta2)*(theta1dot + theta2dot);
    return IMUthetadot + (- xdot*y + ydot*x)/pow(y,2)/(1+pow(x/y,2));
}