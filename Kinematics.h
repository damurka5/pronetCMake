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
    double q[4];

	Kinematics(double x1=0, double y1=0, double x2=2004, double y2=0, double x3=2004, double y3=2004, double x4=0, double y4=2004, double z=2440);
	~Kinematics();

    int calculateInverseKinematics(double x, double y, double z);

    vector<double> getInverseKinematics(double x, double y, double z);
    
private:
    double x1, y1, x2, y2, x3, y3, x4, y4, z;
    
};