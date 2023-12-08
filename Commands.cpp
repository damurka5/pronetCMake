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
            for (int i = 0; i < 4; i++){
                int status = robot->readActualSpeed(i+1, data);
                std::cout << "Servo "<<i+1<<" actual speed: "<<data[0]<<"\n";
                if (status != 0){
                    std::cout << "Error in actual speed\n";
                }

                status = robot->readSetSpeed(i+1, data);
                std::cout << "Servo "<<i+1<<" setted speed: "<<data[0] << "\n";
                if (status != 0){
                    std::cout << "Error in setted speed\n";
                }

                status = robot->readActualPosition(i+1, data);
                std::cout << "Servo " << i + 1 << " actual position: " << data[0] << "\n";
                if (status != 0){
                    std::cout << "Error in position\n";
                }
            }
            std::cin >> command;
        }
        
        if (command == "run5sec") {
            int oneSec = 1000;
            int status = robot->forwardStart(0);
            if (status == 0) std::cout<<"Forward rotation started\n";
            else std::cout<<"Error in rotation\n";
            Sleep(5000);

            status = robot->stopRotation(0);
            if (status == 0) std::cout<<"Stopped\n";
            else std::cout<<"Error in stopping\n"; // do not attempt to change direction without stopping
            //Sleep(1000); // without sleep there is delay 

            std::cin >> command;
            //status = robot->reverseStart(0);
            //if (status == 0) std::cout<<"Reverse rotation started\n";
            //else std::cout<<"Error in rotation\n";
            //Sleep(5000);

            //status = robot->stopRotation(0);
            //if (status == 0) std::cout<<"Stopped\n";
            //else std::cout<<"Error in stopping\n";
            //std::cin >> command;
        }

        //if (command == "test") {
        //    std::vector<std::vector<double>> path;
        //    std::vector<double> zero = { 0, 0, 0 };
        //    std::vector<double> p1 = { 1, 0, 0 };
        //    pathExecution(robot, path, 1);
        //}

    }
};

void Commands::pathExecution(Pronet08* robot, std::vector<std::vector<double>> path, double velocity){
    Kinematics robotKinematics;
    
    // path differentiation on pieces = dl 
    std::vector<std::vector<double>> differentiatedPath;
    differentiatedPath.push_back(path[0]);
    for (int i = 1; i < path.size(); i++){
        double dS = hypot(path[i][0] - path[i-1][0], 
                          path[i][1] - path[i-1][1], 
                          path[i][2] - path[i-1][2]); // distance between two adjacent points in path
        int n;
        n = 1 + (dS/robotKinematics.dl); 
        
        double t = dS/velocity;
        double dt = t/n;

        double dSx = (path[i][0] - path[i-1][0])/n; 
        double dSy = (path[i][1] - path[i-1][1])/n; 
        double dSz = (path[i][1] - path[i-1][1])/n; 

        for (int j = 1; j < n; j++){
            if (j != n-1){
                std::vector<double> point = {dSx * i + path[i-1][0], dSy * i + path[i-1][1], dSz * i + path[i-1][2]};
                differentiatedPath.push_back(point);
            } else {
                std::vector<double> point = {path[i][0], path[i][1], path[i][2]};
                differentiatedPath.push_back(point);
            }
        }
    }
    
    for (int i = 0; i < differentiatedPath.size(); i++){
        std::cout<<"x: "<<differentiatedPath[i][0]<<"\n"
                 <<"y: "<<differentiatedPath[i][0]<<"\n"
                 <<"z: "<<differentiatedPath[i][0]<<"\n";
    }
    

    // calculating q for each piece

    // adding together all differenitated pieces

    // executing the robot
};

