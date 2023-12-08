#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <stdint.h>
#include <cmath>
#include <vector>

#define SAFE_VALUE 1000

class Kinematics
{
public:
    int l[4]; // in mm
    int q_su[4]; // in mm
    int q0_su[] = {26, 18, 34, 30}; //TODO: need to change!!! in servo units
    int l0[] = {3624+SAFE_VALUE, 3624+SAFE_VALUE, 3624+SAFE_VALUE, 3624+SAFE_VALUE}; // in mm
    int dq[4] = {0, 0, 0, 0};
    double dl, t, q_to_l_coeff;
    //double x1=0, double y1=0, double x2=2308, double y2=0, double x3=2308, double y3=2808, double x4=0, double y4=2808, double z=3935
	Kinematics(double x1=0, double y1=0, double x2=2308, double y2=0, double x3=2308, double y3=2808, double x4=0, double y4=2808, double z=3935);
	~Kinematics();

    int calculateInverseKinematics(double x, double y, double z);
    std::vector<double> getInverseKinematics(double x, double y, double z);
    void updateServo_q0_su(int q0[4]);
    void updateServo_q_su(int new_q[4]);
    void updateCableLength(int new_q[4]);
    void setCoeff(double coeff);
    
private:
    double x1, y1, x2, y2, x3, y3, x4, y4, z;
    
};

double hypot(double x, double y, double z);