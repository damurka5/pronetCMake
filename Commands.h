#include "Pronet08.h"
#include "Kinematics.h"
#include <iostream>
#include <string>

#define PULSESREV 131071
#define DEVIDER 364.1
#define FAULT 35

class Commands
{
public:
    Commands();
    ~Commands();

    void startLoop(Pronet08 *robot, std::vector<std::vector<double>> path, double velocity);
    void pointExecution(Pronet08 *robot);
    void pathExecution(Pronet08 *robot, std::vector<std::vector<double>> path, double velocity);
    void pathDifferentiation(std::vector<std::vector<double>> path, double velocity, std::vector<std::vector<double>> &differentiatedPath, std::vector<double> &timeIntervals);
    void setNewZeroPosition(Pronet08 *robot, Kinematics *robotKinematics);
    void cartesianToRobotFrame(std::vector<std::vector<double>> differentiatedPath, std::vector<double> timeIntervals, std::vector<std::vector<int>> &states);
    void executePathTraversal(Pronet08 *robot, std::vector<std::vector<int>> states);
    void checkAllServos(Pronet08 *robot);

private:
    bool exit;
    bool connected;
    bool servoOn[4];
};