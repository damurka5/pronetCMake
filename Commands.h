#include "Pronet08.h"
#include <iostream>
#include <string>

class Commands
{
public:
	Commands();
	~Commands();

    void startLoop(Pronet08* robot);
    
private:
    bool exit;
    bool connected;
    bool servoOn[4];
};