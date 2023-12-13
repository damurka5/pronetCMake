#include "Commands.h"

Commands::Commands()
{
    exit = false;
    connected = false;
    servoOn[0] = false;
    servoOn[1] = false;
    servoOn[2] = false;
    servoOn[3] = false;
};

Commands::~Commands(){

};

void Commands::startLoop(Pronet08 *robot, std::vector<std::vector<double>> path, double velocity)
{
    std::cout << "Enter the command: \n";
    std::string command;
    std::cin >> command;
    while (!exit)
    {
        if (command == "exit")
        {
            std::cout << "Loop is ended\n Closing the connection\n";
            robot->servoOff(0);
            robot->closeSerialPort();
            exit = true;
            break;
        }
        else if (command == "start")
        {
            std::cout << "Loop is started \nEnter the command connect to connect the robot:\n";
            std::cin >> command;
        }
        else if (command == "connect")
        {
            std::cout << "Enter comPort below:\n";
            int comPort;
            std::cin >> comPort;
            robot = new Pronet08(comPort, 9600);
            int status = robot->connectRequest();
            if (status != 0)
            {
                std::cout << "Try to reconnect and enter comPort again or exit:\n";
                std::cin >> command;
            }
            else
            {
                connected = true;
                std::cin >> command;
            }
        }
        else if (command == "servoOn")
        {
            if (!connected)
            {
                std::cout << "Robot is NOT connected\n";
            }
            else
            {
                std::cout << "Turning ON all servos\n";
                int status = robot->servoOn(0);
                if (status != 0)
                {
                    std::cout << "Error in turning ON all servos\n";
                }
                else
                {
                    for (size_t i = 0; i < 4; i++)
                    {
                        servoOn[i] = true;
                    }
                }
            }
            std::cin >> command;
        }
        else if (command == "servoOff")
        {
            if (!connected)
            {
                std::cout << "Robot is NOT connected\n";
            }
            else
            {
                std::cout << "Enter servo num below:\n";
                std::cout << "Turning OFF all servos\n";
                int status = robot->servoOff(0);
                if (status != 0)
                {
                    std::cout << "Error in turning OFF all servos\n";
                }
                else
                {
                    for (int i = 0; i < 4; i++)
                    {
                        servoOn[i] = false;
                    }
                }
            }
            std::cin >> command;
        }
        else if (command == "setSpeed")
        {
            if (!connected)
            {
                std::cout << "Robot is NOT connected\n";
            }
            else
            {
                if (servoOn[1] && servoOn[2] && servoOn[3] && servoOn[4])
                {
                    std::cout << "Enter servo num below:\n";
                    int n;
                    std::cin >> n;
                    std::cout << "Enter speed for servo " << n << "\n";
                    int speed;
                    std::cin >> speed;
                    int status = robot->setSpeed(n, speed);
                    if (status == 0)
                        std::cout << "Speed setted to servo " << n << "\n";
                    else
                        std::cout << "Error in speed setting to servo " << n << "\n";
                }
            }
            std::cin >> command;
        }
        else if (command == "forward")
        {
            if (!connected)
            {
                std::cout << "Robot is NOT connected\n";
            }
            else
            {
                std::cout << "Robot will rotate servo, enter the servo below:\n";
                int n;
                std::cin >> n;
                int status = robot->forwardStart(n);
                if (status == 0)
                    std::cout << "Forward rotation started " << n << "\n";
                else
                    std::cout << "Error in rotation " << n << "\n";
            }
            std::cin >> command;
        }
        else if (command == "reverse")
        {
            if (!connected)
            {
                std::cout << "Robot is NOT connected\n";
            }
            else
            {
                std::cout << "Robot will rotate servo, enter the servo below:\n";
                int n;
                std::cin >> n;
                int status = robot->reverseStart(n);
                if (status == 0)
                    std::cout << "Reverse rotation started " << n << "\n";
                else
                    std::cout << "Error in rotation " << n << "\n";
            }
            std::cin >> command;
        }
        else if (command == "reset")
        {
            int status = robot->resetCommunicationSettings(0);
            if (status != 0)
                std::cout << "Error while reseting\n";
            else
                std::cout << "Reseted\n";
            std::cin >> command;
        }
        else if (command == "stop")
        {
            int status = robot->stopRotation(0);
            if (status != 0)
                std::cout << "Error while stopping\n";
            else
                std::cout << "Stopped\n";
            std::cin >> command;
        }
        else if (command == "checkServo")
        {
            checkAllServos(robot);
            std::cin >> command;
        }
        else if (command == "run5sec")
        {
            int status = robot->forwardStart(0);
            if (status == 0)
                std::cout << "Forward rotation started\n";
            else
                std::cout << "Error in rotation\n";
            Sleep(5000);

            status = robot->stopRotation(0);
            if (status == 0)
                std::cout << "Stopped\n";
            else
                std::cout << "Error in stopping\n"; // do not attempt to change direction without stopping
            // Sleep(1000); // without sleep there is delay

            std::cin >> command;
        }
        else if (command == "test")
        {
            // test to move robot up for 13 mm
            Kinematics robotKinematics;
            robotKinematics.coeff_speed_to_dq = 6.32;
            robotKinematics.l_to_q_coeff = 57.45;
            robotKinematics.dl = 50;

            int dest_mm[3] = {0, 0, 145};

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

            bool reached = false;
            bool servoReached[4] = {false, false, false, false};
            int status;

            std::cout << "Full stop, reached the target\n";
            robot->stopRotation(0);

            std::cin >> command;
        }
        else if (command == "path")
        {
            pathExecution(robot, path, velocity);
            std::cin >> command;
        }
        else if (command == "pointExecution")
        {
            pointExecution(robot);
            std::cin >> command;
        }
        else
        {
            std::cout << "Unknown command, please enter new command\n";
            std::cin >> command;
        }
    }
};

void Commands::pointExecution(Pronet08 *robot)
{
}

void Commands::pathExecution(Pronet08 *robot, std::vector<std::vector<double>> path, double velocity)
{
    Kinematics robotKinematics;
    robotKinematics.coeff_speed_to_dq = 6.32;
    robotKinematics.l_to_q_coeff = 57.45;
    robotKinematics.dl = 200;

    // path differentiation on pieces = dl
    std::vector<std::vector<double>> differentiatedPath;
    std::vector<double> timeIntervals;
    pathDifferentiation(path, velocity, differentiatedPath, timeIntervals); // TODO: check correctness

    // printing values
    for (int i = 0; i < differentiatedPath.size(); i++)
    {
        std::cout << "Point " << i << "\n"
                  << "x: " << differentiatedPath[i][0] << " "
                  << "y: " << differentiatedPath[i][1] << " "
                  << "z: " << differentiatedPath[i][2] << "\n";
    }

    setNewZeroPosition(robot, robotKinematics);

    // calculating q for each piece
    std::vector<std::vector<int>> states; // (q1,q2,q3,q4, w1,w2,w3,w4), q in servo frame, w in servo frame
    cartesianToRobotFrame(differentiatedPath, timeIntervals, states);

    std::cout << "Type yes to execute\n";
    std::string command;
    std::cin >> command;
    if (command != "yes")
        return;

    // executing the robot
    executePathTraversal(robot, states);

    //checkAllServos(robot);
};

void Commands::pathDifferentiation(std::vector<std::vector<double>> path, double velocity, std::vector<std::vector<double>> &differentiatedPath, std::vector<double> &timeIntervals)
{
    differentiatedPath.push_back(path[0]);
    timeIntervals.push_back(0); // time to reach zero position
    for (int i = 1; i < path.size(); i++)
    {
        double dS = hypot(path[i][0] - path[i - 1][0],
                          path[i][1] - path[i - 1][1],
                          path[i][2] - path[i - 1][2]); // distance between two adjacent points in path
        int n;
        n = dS / robotKinematics.dl;

        double t = dS / velocity;
        double dt = t / n;

        double dSx = (path[i][0] - path[i - 1][0]) / n;
        // std::cout << dSx << "\n";
        double dSy = (path[i][1] - path[i - 1][1]) / n;
        double dSz = (path[i][2] - path[i - 1][2]) / n;
        // std::cout << n << "\n";

        for (int j = 1; j <= n; j++)
        {
            if (j != n)
            {
                std::vector<double> point = {dSx * j + path[i - 1][0], dSy * j + path[i - 1][1], dSz * j + path[i - 1][2]};
                differentiatedPath.push_back(point);
                timeIntervals.push_back(dt); // adding dt corresponding to each path slice
            }
            else
            {
                std::vector<double> point = {path[i][0], path[i][1], path[i][2]};
                differentiatedPath.push_back(point);
                timeIntervals.push_back(dt * hypot(point[0] - path[i - 1][0] - dSx * (j - 1), point[1] - path[i - 1][1] - dSy * (j - 1), point[2] - path[i - 1][2] - dSz * (j - 1)) / robotKinematics.dl); // adding last dt (a bit bigger than others)
            }
        }
    }
};

void Commands::setNewZeroPosition(Pronet08 *robot, Kinematics *robotKinematics)
{
    uint16_t data[255] = {
        0,
    };
    int cur_q_su[4] = {13923, 11202, 16636, 15395}; // q0_su

    // Uncomment in Innopark
    for (int i = 0; i < 4; i++)
    {
        int status = robot->readActualPosition(i + 1, data);
        if (status != 0)
            std::cout << "Error in position\n";
        int revolutions = data[0];
        int pulses = data[2] << 16 | data[1];
        int pos = round((revolutions * PULSESREV + pulses) / DEVIDER);
        cur_q_su[i] = pos;
        std::cout << "Servo " << i + 1 << " actual position updated to: " << pos << "\n";
    }

    // should correspond to zero position
    robotKinematics->updateServo_q0_su(cur_q_su);
    int cur_l[4];
    // actual length calculation
    robotKinematics->updateCableLength(cur_l);
    std::cout << "updated\n";
};

void Commands::cartesianToRobotFrame(std::vector<std::vector<double>> differentiatedPath, std::vector<double> timeIntervals, std::vector<std::vector<int>> &states)
{
    for (int i = 0; i < differentiatedPath.size(); i++)
    {
        // convert (x,y,z) to (q1,q2,q3,q4)
        std::vector<int> state;
        std::vector<int> l;
        std::vector<int> q;
        std::vector<int> q_dot;

        // calculating q
        l = robotKinematics.getInverseKinematics(differentiatedPath[i][0], differentiatedPath[i][1], differentiatedPath[i][2]);
        // printing length of cables after inverse kinematics
        for (int j = 0; j < 4; j++)
        {
            std::cout << "l" << j << " is " << l[j] << "\n";
        }

        q = robotKinematics.getQfromL(l);
        for (int j = 0; j < 4; j++)
        {
            state.push_back(q[j]);
        }

        // calculating q_dot
        if (i == 0)
        {
            for (int j = 0; j < 4; j++)
            {
                state.push_back(0);
            }
        }
        else
        {
            // TODO: check negative speed
            for (int j = 0; j < 4; j++)
            {
                int omega_i = (state[j] - states[i - 1][j]) / timeIntervals[i]; // in servo frame (q dot)
                if (omega_i < 0)
                {
                    state.push_back(0.98 * omega_i / robotKinematics.coeff_speed_to_dq);
                }
                else
                {
                    state.push_back(omega_i / robotKinematics.coeff_speed_to_dq); // convertion from servo frame (q dot) to speed in program units
                }
            }
        }
        states.push_back(state);
    }
    std::cout << "states added\n";
    for (int i = 0; i < states.size(); i++)
    {
        std::cout << "Point " << i << "\n";
        for (int j = 0; j < states[i].size(); j++)
        {
            std::cout << "q" << j << ": " << states[i][j] << " ";
        }
        std::cout << "\n";
    }
    for (int i = 0; i < timeIntervals.size(); i++)
    {
        std::cout << timeIntervals[i] << "\n";
    }
};

void Commands::executePathTraversal(Pronet08 *robot, std::vector<std::vector<int>> states)
{
    for (int i = 1; i < states.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (states[i][j + 4] >= 0)
            {
                robot->setSpeed(j + 1, states[i][j + 4]);
            }
            else
            {
                robot->setSpeed(j + 1, -1 * states[i][j + 4]);
            }
        }
        //  check if speed negative
        for (int j = 0; j < 4; j++)
        {
            if (states[i][j + 4] >= 0)
            {
                robot->forwardStart(j + 1);
            }
            else
            {
                robot->reverseStart(j + 1);
            }
        }

        // path execution by time
        Sleep((timeIntervals[i] * 1000) - 29);

        for (int j = 0; j < 4; j++)
        {
            robot->stopRotation(j + 1);
        }
    }
};

void Commands::checkAllServos(Pronet08 *robot)
{
    std::cout << "Info about each servo below:\n";
    uint16_t data[255] = {
        0,
    };
    for (int i = 0; i < 4; i++)
    {
        int status = robot->readActualSpeed(i + 1, data);
        std::cout << "Servo " << i + 1 << " actual speed: " << data[0] << "\n";
        if (status != 0)
            std::cout << "Error in actual speed\n";

        status = robot->readSetSpeed(i + 1, data);
        std::cout << "Servo " << i + 1 << " setted speed: " << data[0] << "\n";
        if (status != 0)
            std::cout << "Error in setted speed\n";

        status = robot->readActualPosition(i + 1, data);

        status = robot->readActualPosition(i + 1, data);
        if (status != 0)
            std::cout << "Error in position\n";
        int revolutions = data[0];
        int pulses = data[2] << 16 | data[1];
        int pos = round((revolutions * PULSESREV + pulses) / DEVIDER);
        std::cout << "Servo " << i + 1 << " actual position: " << pos << "\n";
    }
};