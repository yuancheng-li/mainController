#include <main.h>

using namespace rokae;
using namespace std;

robot::robotArm m_robot;

int main()
{
    string dashes(50, '-');
    string spaces(21, ' ');
    string lslash(17, '>');
    string rslash(17, '<');

    char cmdinfo[400];
    memset(cmdinfo, 0, 400);
    char controllerip[50];
    memset(controllerip, 0, 50);
    
    string robotip = "192.168.0.160";
    std::error_code ec;
    

    bool toQuit = false;
    char cmd;
    cout << endl << dashes << endl;
    cout << spaces << "menu" << spaces;
    cout << endl << dashes << endl;
    sprintf_s(cmdinfo, "\t\tCOMMAND LIST : \t\t       \n"
        "\ta: ------------------------ init           \n"
        "\tq: ------------------------ quit           \n"
        "\tc: ------------------------ start ctrl     \n"
        "\t0: ------------------------ move to init   \n"
        "\te: ------------------------ enable socket  \n");
    cout << setw(50) << left << cmdinfo << endl;

    while (!toQuit)
    {
        cout << lslash << " input  command " << rslash << endl;
        cin >> cmd;
        /*IPRINT("                   %c                \n ", cmd);*/
        
        if (cmd == 'Q' || cmd == 'q')
        {
            toQuit = true;
            break;
        }

        switch (cmd)
        {
            case 'a':
            {
                std::string inputCM;
                std::string inputIP;    
                std::string controllerIP;

                cout << std::endl << dashes << std::endl;
                std::cout << "Input Motion Control Mode:" << std::endl
                    << "'r': RtCommand" << std::endl
                    << "'n': NrtCommand";
                cout << std::endl << dashes << std::endl;
                std::cin >> inputCM;

                cout << std::endl << dashes << std::endl;
                std::cout << "input IP of local controller:" << std::endl
                    << "'d': default IP:192.168.0.100" << std::endl
                    << "else: input IP";
                cout << std::endl << dashes << std::endl;
                std::cin >> inputIP;

                if (inputIP == "d") {
                    controllerIP = "192.168.0.100"; // 使用默认 IP
                }
                else {
                    controllerIP = inputIP; // 使用用户输入的 IP
                }
                m_robot.connect(robotip, controllerIP);
                
                if (inputCM == "r") {   
                    m_robot.robotPtr->setMotionControlMode(MotionControlMode::RtCommand, ec);
                }
                else if (inputCM == "n") {
                    m_robot.robotPtr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
                }
                else {
                    cout << "Wrong Input！" << endl;
                }
                
                break;
            }
            case 'c':
            {
                m_robot.startCtrl(ec);
                break;
            }
            
            case 'q':
                
                break;
        }
            
                

    }

    return 0;
}
