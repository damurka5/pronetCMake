#include "Commands.h"
#include <iostream>

using namespace std;
int main(int argc, char *argv[])
{
   Pronet08* pronet;
   pronet = new Pronet08(0,0);
   Commands* console = new Commands();
   console->startLoop(pronet);
   return 0;
}