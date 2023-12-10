#include "Kinematics.h"

Kinematics::Kinematics(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double z){
    this->x1=x1;
    this->x2=x2;
    this->x3=x3;
    this->x4=x4;
    this->y1=y1;
    this->y2=y2;
    this->y3=y3;
    this->y4=y4;
    this->z=z;
    this->dl = 50; // to differentiate path into small pieces (mm)
};

Kinematics::~Kinematics(){};

double hypot(double x, double y, double z) {
    double res = pow(x, 2) + pow(y, 2) + pow(z, 2);
    return pow(res, 0.5);
}

int Kinematics::calculateInverseKinematics(double x, double y, double z){
    // formula is:
    // q_i = sqrt((x1-Xi)^2 + (y1-Yi)^2 + (z1-Zi)^2)

    int q1 = hypot(this->x1-x, this->y1-y, this->z-z);
    int q2 = hypot(this->x2-x, this->y2-y, this->z-z);
    int q3 = hypot(this->x3-x, this->y3-y, this->z-z);
    int q4 = hypot(this->x4-x, this->y4-y, this->z-z);

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

// get lengths of cables
std::vector<double> Kinematics::getInverseKinematics(double x, double y, double z){
   // formula is:
   // q_i = sqrt((x1-Xi)^2 + (y1-Yi)^2 + (z1-Zi)^2)

   int q1 = hypot(this->x1-x, this->y1-y, this->z-z) + this->q0_su[0];
   int q2 = hypot(this->x2-x, this->y2-y, this->z-z) + this->q0_su[1];
   int q3 = hypot(this->x3-x, this->y3-y, this->z-z) + this->q0_su[2];
   int q4 = hypot(this->x4-x, this->y4-y, this->z-z) + this->q0_su[3];

   std::vector<double> q = {q1, q2, q3, q4};
   return q;
};



// in servo unit
void Kinematics::updateServo_q0_su(int q0[4]){
    for (int i = 0; i < 4; i++){
        this->q0_su[i] = q0[i];
    }
};

//in servo unit
void Kinematics::updateServo_q_su(int new_q[4]){
    for (int i = 0; i < 4; i++){
        this->q_su[i] = new_q[i];
    }
};

// in mm, calculated new length for each cable given servo positions
void Kinematics::updateCableLength(int new_q[4]){
    for (int i = 0; i < 4; i++){
        this->l[i] = this->l0[i] + (this->q0_su[i] - new_q[i]) * this->q_to_l_coeff;
    }
};

// coefficient setter; coefficient translates servo units to mm
void Kinematics::setCoeff(double coeff){
    this->q_to_l_coeff = coeff;
};

std::vector<int> Kinematics::getQfromL(std::vector<double> l){
    std::vector<int> q;
    for (int i = 0; i < 4; i++){
        q_i = this->q0_su[i] + (l[i] - this->l0)/this->q_to_l_coeff;
        q.push_back(q);
    }
    return q;
};