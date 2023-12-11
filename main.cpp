#include "Commands.h"

using namespace std;
int main(int argc, char *argv[])
{
    Pronet08* pronet;
    pronet = new Pronet08(0,0);
    Commands* console = new Commands();
    //console->startLoop(pronet);
    std::vector<double> zero = { 0, 0, 0 };
    std::vector<double> p1 = { 0, 0, 100 };
    std::vector<double> p2 = { 0, 0, 251 };
    std::vector<std::vector<double>> path = {zero, p1};
    //console->pathExecution(pronet, path, 10);
    console->startLoop(pronet, path, 10);

   return 0;
}