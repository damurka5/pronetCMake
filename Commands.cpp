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


void Commands::startLoop(Pronet08* robot, std::vector<std::vector<double>> path, double velocity){
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
        } else if (command == "start") {
            std::cout<<"Loop is started \nEnter the command connect to connect the robot:\n";
            std::cin >> command;
        } else if (command == "connect"){
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
        } else if (command == "servoOn"){
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
        } else if (command == "servoOff"){
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
        } else if (command == "setSpeed"){
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
        } else if (command == "forward") {
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
        } else if (command == "reverse") {
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
        } else if (command == "reset") {
            int status = robot->resetCommunicationSettings(0);
            if (status != 0) std::cout << "Error while reseting\n";
            else std::cout << "Reseted\n";
            std::cin >> command;
        } else if (command == "stop") {
            int status = robot->stopRotation(0);
            if (status != 0) std::cout << "Error while stopping\n";
            else std::cout << "Stopped\n";
            std::cin >> command;
        }
        else if (command == "checkServo") {
            std::cout << "Info about each servo below:\n";
            uint16_t data[255] = { 0, };
            for (int i = 0; i < 4; i++) {
                int status = robot->readActualSpeed(i + 1, data);
                std::cout << "Servo " << i + 1 << " actual speed: " << data[0] << "\n";
                if (status != 0) {
                    std::cout << "Error in actual speed\n";
                }

                status = robot->readSetSpeed(i + 1, data);
                std::cout << "Servo " << i + 1 << " setted speed: " << data[0] << "\n";
                if (status != 0) {
                    std::cout << "Error in setted speed\n";
                }

                status = robot->readActualPosition(i + 1, data);
                // Uncomment in Innopark

                status = robot->readActualPosition(i + 1, data);
                /*std::cout << "Servo " << i + 1 << " actual position: " << data[0] << "\n"*/;
                if (status != 0) std::cout << "Error in position\n";
                int revolutions = data[0];
                int pulses = data[2] << 16 | data[1];
                int pos = round((revolutions * PULSESREV + pulses) / DEVIDER);
                std::cout << "Servo " << i + 1 << " actual position: " << pos << "\n";
            }
            std::cin >> command;
        } else if (command == "run5sec") {
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
        } else if (command == "test") {
            // test to move robot up for 13 mm
            Kinematics robotKinematics;
            double t = 2; // 2 secs to move
            robotKinematics.coeff_speed_to_dq = 6.32; 
            robotKinematics.l_to_q_coeff = 57.45;
            robotKinematics.dl = 50;

            int dest_mm[3] = { 0, 0, 145 };

            std::vector<int> cableLength = robotKinematics.getInverseKinematics(dest_mm[0], dest_mm[1], dest_mm[2]);
            for (int j = 0; j < 4; j++)
            {
                std::cout << "l" << j << " is " << cableLength[j] << "\n";
            }

            std::vector<int> q = robotKinematics.getQfromL(cableLength);
            for (int j = 0; j < 4; j++)
            {
                std::cout << "q" << j << " is " << q[j] << "\n";
            }


            int dest_su[4] = { 9714, 7004, 12672, 11340 };

            bool reached = false;
            bool servoReached[4] = { false, false, false, false };
            int status;
            //robot->forwardStart(0);

            //// while not reached the destination point
            //while (!reached) {
            //    uint16_t data[255] = { 0, };
            //    for (int i = 0; i < 4; i++)
            //    {
            //        if (!servoReached[i]) {
            //            status = robot->readActualPosition(i + 1, data);
            //            /*std::cout << "Servo " << i + 1 << " actual position: " << data[0] << "\n"*/;
            //            if (status != 0) std::cout << "Error in position\n";
            //            int revolutions = data[0];
            //            int pulses = data[2] << 16 | data[1];
            //            int pos = round((revolutions * PULSESREV + pulses) / DEVIDER);
            //            //std::cout << "Servo " << i + 1 << " actual position: " << pos << "\n";
            //            if (pos >= dest_su[i] - FAULT) {
            //                robot->setSpeed(i + 1, 0);
            //                std::cout << "servo " << i + 1 << " reached the point\n";
            //                servoReached[i] = true;
            //            }
            //        }
            //        
            //    }
            //    int c = 0;
            //    for (int i = 0; i < 4; i++)
            //    {
            //        if (servoReached[i]) {
            //            c += 1;
            //        }
            //    }
            //    if (c == 4) {
            //        reached = true;
            //    }
                //if (status != 0) break;
            //}
            std::cout << "Full stop, reached the target\n";
            robot->stopRotation(0);

            std::cin >> command;
        }
        else if (command == "path") {
            pathExecution(robot, path, velocity);
            std::cin >> command;
        }
        else {
            std::cout<<"Unknown command, please enter new command\n";
            std::cin >> command;
        }

    }
};

void Commands::pathExecution(Pronet08* robot, std::vector<std::vector<double>> path, double velocity){
    Kinematics robotKinematics;
    robotKinematics.coeff_speed_to_dq = 6.32;
    robotKinematics.l_to_q_coeff = 57.45;
    robotKinematics.dl = 50;
    
    // path differentiation on pieces = dl 
    std::vector<std::vector<double>> differentiatedPath;
    std::vector<double> timeIntervals;
    differentiatedPath.push_back(path[0]);
    timeIntervals.push_back(0); // time to reach zero position
    for (int i = 1; i < path.size(); i++){
        double dS = hypot(path[i][0] - path[i-1][0], 
                          path[i][1] - path[i-1][1], 
                          path[i][2] - path[i-1][2]); // distance between two adjacent points in path
        int n;
        n = dS/robotKinematics.dl; 
        
        double t = dS/velocity;
        double dt = t/n;

        double dSx = (path[i][0] - path[i-1][0])/n; 
        //std::cout << dSx << "\n";
        double dSy = (path[i][1] - path[i-1][1])/n; 
        double dSz = (path[i][2] - path[i-1][2])/n; 
        //std::cout << n << "\n";

        for (int j = 1; j <= n; j++){
            if (j != n){
                std::vector<double> point = {dSx * j + path[i-1][0], dSy * j + path[i-1][1], dSz * j + path[i-1][2]};
                differentiatedPath.push_back(point);
                timeIntervals.push_back(dt); // adding dt corresponding to each path slice
            } else {
                std::vector<double> point = {path[i][0], path[i][1], path[i][2]};
                differentiatedPath.push_back(point);
                timeIntervals.push_back(dt * hypot(point[0]-dSx, point[1]-dSy, point[2]-dSz)/robotKinematics.dl); // adding last dt (a bit bigger than others)
            }
        }
    }
    
    //printing values
    for (int i = 0; i < differentiatedPath.size(); i++){
        std::cout<<"x: "<<differentiatedPath[i][0]<<"\n"
                 <<"y: "<<differentiatedPath[i][1]<<"\n"
                 <<"z: "<<differentiatedPath[i][2]<<"\n";
    }
    
    // calculating q for each piece
    uint16_t data[255] = { 0, };
    int cur_q_su[4] = { 9081, 6373, 12040, 10708 }; // q0_su

    // Uncomment in Innopark
    //for (int i = 0; i < 4; i++){
    //    int status = robot->readActualPosition(i+1, data);
    //    std::cout << "Servo " << i + 1 << " actual position: " << data[0] << "\n";
    //    if (status != 0) std::cout << "Error in position\n";
    //    int revolutions = data[0];
    //    int pulses = data[2] << 16 | data[1];
    //    int pos = round((revolutions * PULSESREV + pulses) / DEVIDER);
    //    cur_q_su[i] = pos;        
    //}

    // should correspond to zero position
    robotKinematics.updateServo_q0_su(cur_q_su);
    int cur_l[4];
    // actual length calculation
    robotKinematics.updateCableLength(cur_l);
    std::cout << "updated\n";

    std::vector<std::vector<int>> states; // (q1,q2,q3,q4, w1,w2,w3,w4), q in servo frame, w in servo frame

    for (int i = 0; i < differentiatedPath.size(); i++){
        // convert (x,y,z) to (q1,q2,q3,q4)
        std::vector<int> state;
        std::vector<int> l;
        std::vector<int> q;
        std::vector<int> q_dot;

        //calculating q
        l = robotKinematics.getInverseKinematics(differentiatedPath[i][0], differentiatedPath[i][1], differentiatedPath[i][2]);
        //printing length of cables after inverse kinematics
        for (int j = 0; j < 4; j++)
        {
            std::cout << "l" << j << " is " << l[j] << "\n";
        }
        
        q = robotKinematics.getQfromL(l);
        for (int j = 0; j < 4; j++){
            state.push_back(q[j]);
        }
        
        //calculating q_dot
        if (i == 0){
            for (int j = 0; j < 4; j++){
                state.push_back(0);
            }
        } else {
            // TODO: check negative speed
            for (int j = 0; j < 4; j++){
                int omega_i = (state[j] - states[i-1][j])/timeIntervals[i]; // in servo frame (q dot)
                state.push_back(omega_i/robotKinematics.coeff_speed_to_dq); // convertion from servo frame (q dot) to speed in program units
            }
        }
        states.push_back(state);
    }
    std::cout << "states added\n";
    for (int i = 0; i < states.size(); i++){
        std::cout << "Point " << i << "\n";
        for (int j = 0; j < states[i].size(); j++){
            std::cout << "q" << j << ": " << states[i][j] << " ";
        }
        std::cout << "\n";
    }
    for (int i = 0; i < timeIntervals.size(); i++)
    {
        std::cout << timeIntervals[i] << "\n";
    }

    std::string command;
    std::cin>>command; 
    std::cout << "Type yes to execute\n";
    if (command != "yes") return;
    // executing the robot

    for (int i = 1; i < states.size(); i++){
        bool reached = false;
        bool servoReached[4] = { false, false, false, false };
        int status;
        for (int j = 0; j < 4; j++){
            robot->setSpeed(j + 1, states[i][j+4]);
        }
        std::cout<<"Point #"<<i<<" execution\n";
        // robot->forwardStart(0);
        // check if speed negative
        for (int j = 0; j < 4; j++){
            if (states[i][j+4] >= 0){
                robot->forwardStart(j+1);
            } else {
                robot->reverseStart(j+1);
            }
        }
        

        // while not reached the destination point
        while (!reached) {
            uint16_t data[255] = { 0, };
            for (int j = 0; j < 4; j++) {
                if (!servoReached[j]) {
                    status = robot->readActualPosition(j + 1, data);
                    /*std::cout << "Servo " << i + 1 << " actual position: " << data[0] << "\n"*/;
                    if (status != 0) std::cout << "Error in position\n";
                    int revolutions = data[0];
                    int pulses = data[2] << 16 | data[1];
                    int pos = round((revolutions * PULSESREV + pulses) / DEVIDER);
                    //std::cout << "Servo " << i + 1 << " actual position: " << pos << "\n";
                    if (pos >= states[i][j] - FAULT && pos <= states[i][j] + FAULT) { // if servo lies in between -FAULT to +FAULT it has reached the point 
                        robot->setSpeed(j + 1, 0);
                        std::cout << "servo " << j + 1 << " reached the point\n";
                        servoReached[j] = true;
                    }
                }
            }
            int c = 0;
            for (int j = 0; j < 4; j++){
                if (servoReached[j]) {
                    c += 1;
                }
            }
            if (c == 4) {
                reached = true;
            }
            // if (status != 0) break;
        }
        robot->stopRotation(0);
    }
};
