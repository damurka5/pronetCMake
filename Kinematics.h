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
    int q_su[4]; // in servo units
    int q0_su[4] = {9081, 6373, 12040, 10708}; // in servo units
    int l0[4] = {3780, 3780, 3780, 3780 }; // in mm
    int dq[4] = {0, 0, 0, 0};
    double dl, t, l_to_q_coeff, speedRPM, speedL, speedQ, coeff_speed_to_dq;
    //double x1=0, double y1=0, double x2=2308, double y2=0, double x3=2308, double y3=2808, double x4=0, double y4=2808, double z=3935
	Kinematics(double x1=-1154, double y1=-1404, double x2=1154, double y2=-1404, double x3=1154, double y3=1404, double x4=-1154, double y4=1404, double z=3315);
	~Kinematics();

    int calculateInverseKinematics(double x, double y, double z);
    std::vector<int> getInverseKinematics(double x, double y, double z);
    void updateServo_q0_su(int q0[4]);
    void updateServo_q_su(int new_q[4]);
    void updateCableLength(int new_q[4]);
    void setCoeff(double coeff);
    std::vector<int> getQfromL(std::vector<int> l);
    
private:
    double x1, y1, x2, y2, x3, y3, x4, y4, z;
    
};

double hypot(double x, double y, double z);