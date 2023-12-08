#include "Pronet08.h"
#include "Kinematics.h"
#include <iostream>
#include <string>

class Commands
{
public:
	Commands();
	~Commands();

    void startLoop(Pronet08* robot);

    void pathExecution(Pronet08* robot, std::vector<std::vector<double>> path, double velocity);
    
private:
    bool exit;
    bool connected;
    bool servoOn[4];
};