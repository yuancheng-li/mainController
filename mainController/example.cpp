/**
 * @file move_example.cpp.
 * @brief ��ʵʱ�˶�ָ��. ���ݻ��ͺ�����ϵ�Ĳ�ͬ, ��ʾ���еĵ�λ��һ���ɴ�, �����ӿ�ʹ�÷����Ĳο�
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include <cmath>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "rokae/print_helper.hpp"
#include <corecrt_math_defines.h>

using namespace std;
using namespace rokae;
std::ostream& os = std::cout;

namespace Predefines {
    // ******   ��קλ��   ******
    // xMateEr3, xMateEr7
    const std::vector<double> ErDragPosture = { 0, M_PI / 6, M_PI / 3, 0, M_PI_2, 0 };
    // xMateEr3 Pro, xMateEr7 Pro
    const std::vector<double> ErProDragPosture = { 0, M_PI / 6, 0, M_PI / 3, 0, M_PI_2, 0 };
    // xMateCR
    const std::vector<double> CrDragPosture{ 0, M_PI / 6, -M_PI_2, 0, -M_PI / 3, 0 };

    // Ĭ�Ϲ��߹���
    Toolset defaultToolset;
}
/**
 * @brief ��ӡ�˶�ִ����Ϣ
 */
void printInfo(const rokae::EventInfo& info) {
    using namespace rokae::EventInfoKey::MoveExecution;
    print(std::cout, "[�˶�ִ����Ϣ] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "�����: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES" : "NO", std::any_cast<error_code>(info.at(Error)),
        std::any_cast<std::string>(info.at(Remark)));
}

/**
 * @brief �ȴ��˶����� - ͨ����ѯ·��ID��·������Ƿ�����ɵķ�ʽ
 */
void waitForFinish(BaseRobot& robot, const std::string& traj_id, int index) {
    using namespace rokae::EventInfoKey::MoveExecution;
    error_code ec;
    while (true) {
        auto info = robot.queryEventInfo(Event::moveExecution, ec);
        auto _id = std::any_cast<std::string>(info.at(ID));
        auto _index = std::any_cast<int>(info.at(WaypointIndex));
        if (auto _ec = std::any_cast<error_code>(info.at(Error))) {
            print(std::cout, "·��", _id, ":", _index, "����:", _ec.message());
            return;
        }
        if (_id == traj_id && _index == index) {
            if (std::any_cast<bool>(info.at(ReachTarget))) {
                print(std::cout, "·��", traj_id, ":", index, "�����");
            }
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

/**
 * @brief �ȴ��˶����� - ͨ����ѯ��е���Ƿ����˶��еķ�ʽ
 */
void waitRobot(BaseRobot& robot, bool& running) {
    running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robot.operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) {
            running = false;
        }
    }
}

/**
 * @brief �¼����� - ģ�ⷢ����ײ��ȴ�5���ϵ粢��������
 */
void recoverFromCollision(BaseRobot& robot, const rokae::EventInfo& info) {
    using namespace rokae::EventInfoKey;
    bool isCollided = std::any_cast<bool>(info.at(Safety::Collided));
    print(std::cout, "Collided:", isCollided);
    if (isCollided) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        error_code ec;
        robot.setPowerState(true, ec);
        robot.moveStart(ec);
        print(std::cout, "Recovered from collision");
    }
}

/**
 * @brief �ϸ���ѭ����������(conf data)��ֱ���˶�����λ���û���xMateER3
 */
void moveWithForcedConf(xMateRobot& robot) {
    Toolset default_toolset;
    error_code ec;
    bool running;
    std::string id;
    robot.setToolset(default_toolset, ec);
    robot.setDefaultSpeed(200, ec);
    robot.setDefaultZone(5, ec);

    print(std::cout, "�˶�����קλ��");
    robot.executeCommand({ MoveAbsJCommand(Predefines::ErDragPosture) }, ec);
    waitRobot(robot, running);

    CartesianPosition cartesian_position0({ 0.786, 0, 0.431, M_PI, 0.6, M_PI });
    CartesianPosition cartesian_position1({ 0.786, 0, 0.431, M_PI, 0.98, M_PI });


    MoveJCommand j0({ cartesian_position0 }), j1({ cartesian_position1 });
    MoveLCommand l0({ cartesian_position0 }), l1({ cartesian_position1 });

    // ���ϸ���ѭ���������ݣ� MoveL & MoveJ ���ѡȡ��ǰ��Ƕ������
    print(std::cout, "��ʼMoveJ");
    robot.moveAppend({ j0, j1 }, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, 1);

    // ��ѭ���������ݣ� ʹ��conf������⣬��ʱMoveL�������ʧ��
    robot.setDefaultConfOpt(true, ec);
    print(std::cerr, ec);
    cartesian_position1.confData = { -1,1,-1,0,1,0,0,2 };
    l1.target = cartesian_position1;
    j1.target = cartesian_position1;

    print(std::cout, "��ʼMoveJ");
    robot.moveAppend({ j0, j1 }, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, 1);

    print(std::cout, "�˶�����קλ��");
    robot.executeCommand({ MoveAbsJCommand(Predefines::ErDragPosture) }, ec);
    waitRobot(robot, running);

    print(std::cout, "��ʼMoveL");
    robot.moveAppend({ l0, l1 }, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, 1);
    robot.setDefaultConfOpt(false, ec);
}

/**
 * @brief ʾ�� - �ѿ�����λ����ƫ�� & �˶�����ͣ�����; ��λ���û���xMateEr7 Pro
 */
void cartesianPointWithOffset(BaseRobot& robot) {
    error_code ec;

    std::array<double, 6> pos = { -0.571, -0.15, 0.5568, -M_PI, 0.573, -M_PI };
    std::array<double, 6> offset_z = { 0, 0, 0.2, 0, 0, 0 };

    MoveLCommand moveL1(pos, 500, 5), moveL2(pos, 800, 0);
    // ��Թ�������ϵZ+ƫ��0.2m
    moveL2.offset = { CartesianPosition::Offset::offs, offset_z };

    MoveJCommand moveJ1(pos, 200, 0), moveJ2(pos, 1000, 80);
    // ��Թ�������ϵZ+ƫ��0.2m
    moveJ2.offset = { CartesianPosition::Offset::relTool, offset_z };

    // ִ����4����λ
    robot.executeCommand({ moveL1, moveL2 }, ec);
    robot.executeCommand({ moveJ1, moveJ2 }, ec);

    std::thread input([&] {
        int c{};
        print(os, "[p]��ͣ [c]���� [q]�˳�");
        while (c != 'q') {
            c = getchar();
            switch (c) {
            case 'p':
                robot.stop(ec);
                print(std::cerr, ec); break;
            case 'c':
                robot.moveStart(ec);
                print(std::cerr, ec); break;
            default: break;
            }
        }
        });
    input.join();
    robot.moveReset(ec);
}

/**
 * @brief �������˶������û���: xMate3
 */
void spiralMove(rokae::xMateRobot& robot) {
    error_code ec;
    std::string id;
    rokae::Toolset default_toolset = {};
    robot.setToolset(default_toolset, ec);

    // �������յ���̬, ֻ�õ�rpy, xyzֵ����
    rokae::CartesianPosition cart_target({ 0, 0, 0, 2.967, -0.2, 3.1415 }),
        cart_target1({ 0, 0, 0, -2.787577,0.1639,-2.9 });
    rokae::MoveAbsJCommand absjcmd({ 0.0,0.22150561307150393,1.4779577696969546,0.0,1.2675963456219013,0.0 });

    // ������1: ��ʼ�뾶0.01m, �뾶�仯����0.0005m/rad, ��ʱ����ת720�㣬�ٶ�v500
    rokae::MoveSPCommand spcmd1({ cart_target, 0.01, 0.0005, M_PI * 4, false, 500 }),
        // ������2: ��ʼ�뾶0.05m, �뾶�仯����0.001m/rad, ˳ʱ����ת360�㣬�ٶ�v100
        spcmd2({ cart_target1, 0.05, 0.001, M_PI * 2, true, 100 });

    std::vector<rokae::MoveSPCommand> spcmds = { spcmd1, spcmd2 };
    robot.moveAppend({ absjcmd }, id, ec);
    robot.moveAppend(spcmds, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, (int)spcmds.size() - 1);
}


/**
 * @brief ʾ�� - ���������˶� & ������ײ����ָ��˶�, ��λ���û���xMateER3 Pro
 */
void redundantMove(xMateErProRobot& robot) {
    error_code ec;
    std::string id;

    // ����ʾ��ʹ��Ĭ�Ϲ��߹���, �ٶ�v500, ת����fine
    Toolset defaultToolset;
    robot.setToolset(defaultToolset, ec);
    robot.setDefaultSpeed(500, ec);
    robot.setDefaultZone(0, ec);

    // ��ѡ: ������ײ����¼��ص�����
    robot.setEventWatcher(Event::safety, [&](const EventInfo& info) {
        recoverFromCollision(robot, info);
        }, ec);


    MoveAbsJCommand moveAbsj({ 0, M_PI / 6, 0, M_PI / 3, 0, M_PI_2, 0 });
    // ** 1) ��۽��˶� **
    MoveLCommand moveL1({ 0.562, 0, 0.432, M_PI, 0, -M_PI });
    moveL1.target.elbow = 1.45;
    robot.moveAppend({ moveAbsj }, id, ec);
    robot.moveAppend({ moveL1 }, id, ec);
    moveL1.target.elbow = -1.51;
    robot.moveAppend({ moveL1 }, id, ec);
    robot.moveStart(ec);
    // ���һ��moveAppend()����һ��ָ���index = 0
    waitForFinish(robot, id, 0);

    // ** 2) 60��۽�Բ�� **
    CartesianPosition circle_p1({ 0.472, 0, 0.342, M_PI, 0, -M_PI }),
        circle_p2({ 0.602, 0, 0.342, M_PI, 0, -M_PI }),
        circle_a1({ 0.537, 0.065, 0.342, M_PI, 0, -M_PI }),
        circle_a2({ 0.537, -0.065, 0.342, M_PI, 0, -M_PI });
    // �۽Ƕ���60��
    circle_p1.elbow = M_PI / 3;
    circle_p2.elbow = M_PI / 3;
    circle_a1.elbow = M_PI / 3;
    circle_a2.elbow = M_PI / 3;

    MoveLCommand moveL2(circle_p1);
    robot.moveAppend({ moveL2 }, id, ec);
    MoveCCommand moveC1(circle_p2, circle_a1), moveC2(circle_p1, circle_a2);
    std::vector<MoveCCommand> movec_cmds = { moveC1, moveC2 };
    robot.moveAppend(movec_cmds, id, ec);
    robot.moveStart(ec);
    // ���һ��moveAppend()����2��ָ�����Ҫ�ȴ��ڶ�������ɺ󷵻أ�indexΪ�ڶ�������±�
    waitForFinish(robot, id, (int)movec_cmds.size() - 1);
}

/**
 * @brief xMateSR/xMateCR����������ܣ���ʽ������4�ᡣʾ�����û���xMateCR7
 */
void avoidSingularityMove(rokae::xMateRobot& robot) {
    error_code ec;
    std::string id;
    bool running;
    robot.setToolset(Predefines::defaultToolset, ec);

    // ���˶�����ʼλ��
    robot.executeCommand({ MoveAbsJCommand({0.453,0.539,-1.581,0.0,0.026,0}) }, ec);
    waitRobot(robot, running);

    std::vector<rokae::MoveLCommand> cmds = {
      MoveLCommand({0.66675437164302165, -0.23850040314585069, 0.85182031,-3.1415926535897931, 1.0471975511965979, 3.01151}),
      MoveLCommand({0.66675437164302154, 0.15775146321850292, 0.464946,-3.1415926535897931, 1.0471975511965979, -2.6885547129789127})
    };

    // ����������ģʽ, �ᱨ�����˶���Χ
    robot.setAvoidSingularity(false, ec);
    robot.moveAppend(cmds, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, (int)cmds.size() - 1);

    // ���������ģʽ����λ�ɴ�
    robot.setAvoidSingularity(true, ec);
    std::cerr << ec;
    print(std::cout, "�����ܹ���", robot.getAvoidSingularity(ec) ? "��" : "�ر�");
    robot.moveReset(ec);

    robot.moveAppend(cmds, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, (int)cmds.size() - 1);
    robot.setAvoidSingularity(false, ec);
}

/**
 * @brief ʾ�� - ʹ�ù��߹�������ϵ
 */
void moveInToolsetCoordinate(BaseRobot& robot) {
    error_code ec;
    std::string id;
    // Ĭ�ϵĹ��߹��� tool0, wobj0
    auto currentToolset = robot.setToolset("tool0", "wobj0", ec);
    print(os, "��ǰ���߹�����", currentToolset);

    MoveAbsJCommand moveAbs({ 0, M_PI / 6, -M_PI_2, 0, -M_PI / 3, 0 });
    robot.moveAppend({ moveAbs }, id, ec);
    MoveLCommand movel1({ 0.563, 0, 0.432, M_PI, 0, M_PI }, 1000, 100);
    MoveLCommand movel2({ 0.33467, -0.095, 0.51, M_PI, 0, M_PI }, 1000, 100);
    robot.moveAppend({ movel1, movel2 }, id, ec);
    robot.moveStart(ec);
    bool moving = true;
    waitRobot(robot, moving);

#if 0
    // ������ִ����movel1��movel2, ��Ҫ�л������߹���, ��ִ�к�����˶�ָ��
    // ���ù��߹�����ʽ1: ֱ���趨
    Toolset toolset1;
    toolset1.load.mass = 2; // ����2kg
    toolset1.ref = { {0.1, 0.1, 0}, {0, 0, 0} }; // �ⲿ�ο�����ϵ��X+0.1m, Y+0.1m
    toolset1.end = { { 0, 0, 0.01}, {0, M_PI / 6, 0} }; // ĩ�����꣬Z+0.01m, Ry+30��
    robot.setToolset(toolset1, ec);

    // ���ù��߹�����ʽ2: ʹ���Ѵ����Ĺ��߹���tool1, wobj1
    robot.setToolset("tool1", "wobj1", ec);
    MoveLCommand movel3({ 0.5, 0, 0.4, M_PI, 0, M_PI }, 1000, 100);
    robot.moveAppend({ movel3 }, id, ec);
    robot.moveStart(ec);
#endif
}

/**
 * @brief ʾ�� - �˶��е����˶�����
 */
void adjustSpeed(BaseRobot& robot) {
    error_code ec;
    std::string id;
    double scale = 0.5;
    robot.adjustSpeedOnline(scale, ec); // ������ʼ�ٶȱ���Ϊ50%

    // ʾ����: ��cmd1��cmd2������λ֮���˶�
    rokae::MoveAbsJCommand cmd1({ 0, 0, 0, 0, 0, 0 }), cmd2({ 1.5, 1.5,1.5,1.5,1.5,1.5 });
    robot.moveAppend({ cmd1, cmd2,cmd1,cmd2,cmd1,cmd2,cmd1,cmd2 }, id, ec);
    robot.moveStart(ec);
    bool running = true;

    // ��ȡ��������
    std::thread readInput([&] {
        while (running) {
            auto ch = std::getchar();
            if (ch == 'a') {
                if (scale < 0.1) { print(std::cerr, "�Ѵﵽ1%"); continue; }
                scale -= 1e-1;
            }
            else if (ch == 'd') {
                if (scale > 1) { print(std::cerr, "�Ѵﵽ100%"); continue; }
                scale += 1e-1;
            }
            else { continue; }
            robot.adjustSpeedOnline(scale, ec);
            print(os, "����Ϊ", scale);
        }
        });
    print(os, "�����˿�ʼ�˶�, �밴[a]��С�ٶ� [d]�����ٶ�, ����Ϊ10%");

    // �ȴ��˶�����
    waitRobot(robot, running);
    readInput.join();
}

/**
 * @brief ʾ�� - ��������������(confData)������������, ��λ���û���xMateCR7
 */
void multiplePosture(xMateRobot& robot) {
    error_code ec;
    std::string id;

    // ����ʾ��ʹ��Ĭ�Ϲ��߹���
    Toolset defaultToolset;
    robot.setToolset(defaultToolset, ec);

    MoveJCommand moveJ({ 0.2434, -0.314, 0.591, 1.5456, 0.314, 2.173 });
    // ͬ����ĩ��λ�ˣ�confData��ͬ����Ƕ�Ҳ��ͬ
    moveJ.target.confData = { -67, 100, 88, -79, 90, -120, 0, 0 };
    robot.moveAppend({ moveJ }, id, ec);
    moveJ.target.confData = { -76, 8, -133, -106, 103, 108, 0, 0 };
    robot.moveAppend({ moveJ }, id, ec);
    moveJ.target.confData = { -70, 8, -88, 90, -105, -25, 0, 0 };
    robot.moveAppend({ moveJ }, id, ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, 0);
}

//int main() {
//    try {
//        using namespace rokae;
//
//        // *** 1. ���ӻ����� ***
//        std::string ip = "192.168.0.160";
//        std::error_code ec;
//        rokae::xMateRobot robot(ip); // ****   xMate 6-axis
//
//        // *** 2. �л����Զ�ģʽ���ϵ� ***
//        robot.setOperateMode(OperateMode::automatic, ec);
//        robot.setPowerState(true, ec);
//
//        // *** 3. ����Ĭ���˶��ٶȺ�ת���� ***
//        robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
//        robot.setDefaultZone(50, ec); // ��ѡ������Ĭ��ת����
//        robot.setDefaultSpeed(200, ec); // ��ѡ������Ĭ���ٶ�
//
//        // ��ѡ�������˶�ָ��ִ����ɺʹ�����Ϣ�ص�
//        robot.setEventWatcher(Event::moveExecution, printInfo, ec);
//
//        // *** 4. �˶�ʾ������ ***
//        //multiplePosture(robot);
//
//        robot.setPowerState(false, ec);
//        robot.disconnectFromRobot(ec);
//    }
//    catch (const std::exception& e) {
//        print(std::cerr, e.what());
//    }
//    return 0;
//}