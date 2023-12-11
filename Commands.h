#include "Pronet08.h"
#include "Kinematics.h"
#include <iostream>
#include <string>

#define PULSESREV 131071
#define DEVIDER 364.1
#define FAULT 25

class Commands
{
public:
	Commands();
	~Commands();

    void startLoop(Pronet08* robot, std::vector<std::vector<double>> path, double velocity);

    void pathExecution(Pronet08* robot, std::vector<std::vector<double>> path, double velocity);
    
private:
    bool exit;
    bool connected;
    bool servoOn[4];
};