#include "Commands.h"


Commands::Commands(){
    exit = false;
};

Commands::~Commands(){};


void Commands::startLoop(Pronet08* robot){
    std::cout << "Enter the command: \n";
    std::string command;
    std::cin >> command;
    while (!exit){
        if (command == "exit") {
            std::cout<<"Loop is ended\n Closing the connection\n";
            break;
        };

        if (command == "start") {
            std::cout<<"Loop is started \nEnter the command connect to connect the robot";
            std::cin >> command;
        };

        if (command == "connect"){
            std::cout<<"Enter comPort below:\n";
            int comPort;
            cin>>comPort;
            int status = robot->connectRequest(comPort, 9600);
            if (status != 0){
                std::cout<<"Try to reconnect and enter comPort again or exit:\n";
                cin>>comPort;
            } 
        }

    }
};