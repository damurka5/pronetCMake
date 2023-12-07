#include "Kinematics.h"

Kinematics::Kinematics(double x1=0, double y1=0, double x2=2004, double y2=0, double x3=2004, double y3=2004, double x4=0, double y4=2004, double z=2440){
    this->x1=x1;
    this->x2=x2;
    this->x3=x3;
    this->x4=x4;
    this->y1=y1;
    this->y2=y2;
    this->y3=y3;
    this->y4=y4;
    this.z=z;
};

Kinematics::~Kinematics(){};

int Kinematics::calculateInverseKinematics(double x, double y, double z){
    // formula is:
    // q_i = sqrt((x1-Xi)^2 + (y1-Yi)^2 + (z1-Zi)^2)

    q1 = std::hypot(this->x1-x, this->y1-y, this->z-z);
    q2 = std::hypot(this->x2-x, this->y2-y, this->z-z);
    q3 = std::hypot(this->x3-x, this->y3-y, this->z-z);
    q4 = std::hypot(this->x4-x, this->y4-y, this->z-z);

    this->q[0] = q1;
    this->q[1] = q2;
    this->q[2] = q3;
    this->q[3] = q4;

    int res = 1;
    for (int i = 0; i < 4; i++){
        if (this->q[i] < SAFE_VALUE){
            res = 0;
        }
    }
    
    return res;
};

vector<double> Kinematics::getInverseKinematics(double x, double y, double z){
    // formula is:
    // q_i = sqrt((x1-Xi)^2 + (y1-Yi)^2 + (z1-Zi)^2)

    q1 = std::hypot(this->x1-x, this->y1-y, this->z-z);
    q2 = std::hypot(this->x2-x, this->y2-y, this->z-z);
    q3 = std::hypot(this->x3-x, this->y3-y, this->z-z);
    q4 = std::hypot(this->x4-x, this->y4-y, this->z-z);

    vector<double> q = {q1, q2, q3, q4};
    return q;
};