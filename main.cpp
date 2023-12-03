#include "Pronet08.h"
#include <iostream>

using namespace std;
int main(int argc, char *argv[])
{
   cout<<5<<"\n";
   Pronet08* pronet;
   pronet = new Pronet08(0,0);
   pronet->connectRequest();
   //cout << pronet;
   return 0;
}