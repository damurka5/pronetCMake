#include "Commands.h"


Commands::Commands(){
    exit = false;
    connected = false;
    servoOn[0] = false;
    servoOn[1] = false;
    servoOn[2] = false;
    servoOn[3] = false;
};

Commands::~Commands(){

};


void Commands::startLoop(Pronet08* robot){
    std::cout << "Enter the command: \n";
    std::string command;
    std::cin >> command;
    while (!exit){
        if (command == "exit") {
            std::cout<<"Loop is ended\n Closing the connection\n";
            robot->servoOff(0);
            robot->closeSerialPort();
            exit = true;
            break;
        };

        if (command == "start") {
            std::cout<<"Loop is started \nEnter the command connect to connect the robot:\n";
            std::cin >> command;
        };

        if (command == "connect"){
            std::cout<<"Enter comPort below:\n";
            int comPort;
            std::cin>>comPort;
            robot = new Pronet08(comPort, 9600);
            int status = robot->connectRequest();
            if (status != 0){
                std::cout<<"Try to reconnect and enter comPort again or exit:\n";
                std::cin>>command;
            } else {
                connected = true;
                std::cin>>command;
            }
        }

        if (command == "servoOn"){
            if (!connected){
                std::cout<<"Robot is NOT connected\n";
            } else {
                std::cout<<"Enter servo num below:\n";
                int n;
                std::cin>>n;
                if (n != 0) {
                    std::cout<<"Turning ON servo "<<n<<"\n";
                    int status = robot->servoOn(n);
                    if (status != 0) std::cout<<"Error in turning ON servo "<<n<<"\n";
                    else servoOn[n-1] = true;
                } else {
                    std::cout<<"Turning ON all servos\n";
                    int status = robot->servoOn(n);
                    if (status != 0){
                        std::cout<<"Error in turning ON all servos\n";
                    } else {
                        for (size_t i = 0; i < 4; i++)
                        {
                            servoOn[i] = true;
                        }
                        
                    }
                }
            }
            std::cin>>command;
        }

        if (command == "servoOff"){
            if (!connected){
                std::cout<<"Robot is NOT connected\n";
            } else {
                std::cout<<"Enter servo num below:\n";
                int n;
                std::cin>>n;
                if (n != 0) {
                    std::cout<<"Turning OFF servo "<<n<<"\n";
                    int status = robot->servoOff(n);
                    if (status != 0) std::cout<<"Error in turning OFF servo "<<n<<"\n";
                    else servoOn[n-1] = false;
                } else {
                    std::cout<<"Turning OFF all servos\n";
                    int status = robot->servoOff(n);
                    if (status != 0){
                        std::cout<<"Error in turning OFF all servos\n";
                    } else {
                        for (size_t i = 0; i < 4; i++)
                        {
                            servoOn[i] = false;
                        }
                    }
                }
            }
            std::cin>>command;
        }

        if (command == "setSpeed"){
            if (!connected){
                    std::cout<<"Robot is NOT connected\n";
                } else {
                    if (servoOn[1] && servoOn[2] && servoOn[3] && servoOn[4]){
                        std::cout<<"Enter servo num below:\n";
                        int n;
                        std::cin>>n;
                        std::cout<<"Enter speed for servo "<<n<<"\n";
                        int speed;
                        std::cin>>speed;
                        int status = robot->setSpeed(n, speed);
                        if (status == 0) std::cout<<"Speed setted to servo "<<n<<"\n";
                        else std::cout<<"Error in speed setting to servo "<<n<<"\n";
                    }
                }
            std::cin>>command;
        }

        if (command == "forward") {
             if (!connected){
                    std::cout<<"Robot is NOT connected\n";
                } else {
                    std::cout<<"Robot will rotate servo, enter the servo below:\n";
                    int n;
                    std::cin>>n;
                    int status = robot->forwardStart(n);
                    if (status == 0) std::cout<<"Forward rotation started "<<n<<"\n";
                    else std::cout<<"Error in rotation "<<n<<"\n";
                }
             std::cin >> command;
        }

        if (command == "reverse") {
             if (!connected){
                    std::cout<<"Robot is NOT connected\n";
                } else {
                    std::cout<<"Robot will rotate servo, enter the servo below:\n";
                    int n;
                    std::cin>>n;
                    int status = robot->reverseStart(n);
                    if (status == 0) std::cout<<"Reverse rotation started "<<n<<"\n";
                    else std::cout<<"Error in rotation "<<n<<"\n";
                }
             std::cin >> command;
        }

        if (command == "reset") {
            int status = robot->resetCommunicationSettings(0);
            if (status != 0) std::cout << "Error while reseting\n";
            else std::cout << "Reseted\n";
            std::cin >> command;
        }

        if (command == "stop") {
            int status = robot->stopRotation(0);
            if (status != 0) std::cout << "Error while stopping\n";
            else std::cout << "Stopped\n";
            std::cin >> command;
        }

        if (command == "checkServo") {
            std::cout << "Info about each servo below:\n";
            uint16_t data[255] = { 0, };
            for (size_t i = 0; i < 4; i++){
                int status = robot->readActualSpeed(i+1, data);
                std::cout << "Servo "<<i+1<<" actual speed: "<<data[0];
                if (status != 0){
                    std::cout << "Error in actual speed\n";
                }

                int status = robot->readSetSpeed(i+1, data);
                std::cout << "Servo "<<i+1<<" setted speed: "<<data[0];
                if (status != 0){
                    std::cout << "Error in setted speed\n";
                }

                int status = robot->readActualPosition(i+1, data);
                std::cout << "Servo "<<i+1<<" actual position: "<<data[0];
                if (status != 0){
                    std::cout << "Error in position\n";
                }
            }
            std::cin >> command;
        }
        
        if (command == "run5sec") {
            int oneSec = 1000000;
            int status = robot->forwardStart(0);
            if (status == 0) std::cout<<"Forward rotation started\n";
            else std::cout<<"Error in rotation\n";
            Sleep(2*oneSec);

            int status = robot->stopRotation(0);
            if (status == 0) std::cout<<"Stopped\n";
            else std::cout<<"Error in stopping\n";
            Sleep(oneSec);

            int status = robot->reverseStart(0);
            if (status == 0) std::cout<<"Reverse rotation started\n";
            else std::cout<<"Error in rotation\n";
            Sleep(2*oneSec);

            int status = robot->stopRotation(0);
            if (status == 0) std::cout<<"Stopped\n";
            else std::cout<<"Error in stopping\n";
        }

    }
};

