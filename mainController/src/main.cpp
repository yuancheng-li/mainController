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
            m_robot.stopCtrl_n(ec);
            toQuit = true;
            break;
        }

        std::string inputCM;
        std::string inputIP;

        switch (cmd)
        {
            case 'a':
            {
                std::string controllerIP;

                /* 输入控制模式（实时/非实时 */ 
                cout << std::endl << dashes << std::endl;
                std::cout << "Input Motion Control Mode:" << std::endl
                    << "'r': RtCommand" << std::endl
                    << "'n': NrtCommand";
                cout << std::endl << dashes << std::endl;
                std::cin >> inputCM;
                /* 输入控制器IP */
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
                m_robot.connect(robotip, controllerIP);  // 连接机器人
                m_robot.init();  // 切换到自动模式

                /* 设置运动控制模式 */
                if (inputCM == "r") {   
                    m_robot.robotPtr->setMotionControlMode(MotionControlMode::RtCommand, ec);
                }
                else if (inputCM == "n") {
                    m_robot.robotPtr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
                    m_robot.robotPtr->moveReset(ec);
                    m_robot.robotPtr->setDefaultZone(50, ec);
                    m_robot.robotPtr->setDefaultSpeed(500, ec);
                }
                else {
                    cout << "Wrong Input！" << endl;
                }
                
                break;
            }
            case 'c':
            {
                if (inputCM == "r") {
                    m_robot.startCtrl(ec);
                }
                else if (inputCM == "n") {
                    m_robot.startCtrl_n(ec);
                }
                
                break;
            }

        }
            
                

    }

    return 0;
}
