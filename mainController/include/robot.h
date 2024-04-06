#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <iostream>
#include <cmath>
#include <corecrt_math_defines.h>
#include <thread>
#include "rokae/robot.h"
#include "rokae/print_helper.hpp"
#include "globaldefine.h"
#include <stdio.h>
#include <iomanip>

namespace robot
{
	using namespace std;
	using namespace Eigen;

    enum SM_STATE
    {
        RUNNING = 0x01,
        STOPPED = 0x00
    };

    enum SM_ACTION
    {
        NONE = 0x00,
        ENABLE_MOTORS = 0x01,
        DISABLE_MOTORS = 0x02,
        BEGIN_MOTION = 0x03,
        STOP_MOTION = 0x04
    };

    enum MOTION_TYPE
    {
        JOINT_RML = 0x01,
        CARTESIAN_RML_6D = 0x03,
        CARTESIAN_RML_TRANSLATION = 0x04,
        CARTESIAN_RML_ROTATION = 0x05,
        TRAJ_OFFLINE = 0x06,
        CIRCLE = 0x07,
        TEST_LINE = 0x08,
        TEST_CIRCLE = 0x09
    };

    struct StateMachine
    {
        int action;
        int state;
        int motiontype;
    } m_SM;


    class robotArm
    {
    public:
        robotArm();
        ~robotArm();

#ifdef xMateErPro
        std::unique_ptr<rokae::xMateErProRobot> robotPtr;
#endif

        std::shared_ptr<rokae::RtMotionControlCobot<DoF>> rtCon;
        bool m_stopCtrl;
        float m_TStep;
        
        void connect(const std::string& robotIP, const std::string& controllerIP);
        void init();

        /* 创建运动控制线程开启核心状态机 */
        void startCtrl(std::error_code ec);

        /* 停止运动控制线程的状态机循环 */
        void stopCtrl();

        /* 开始运动,设置运动类型 */
        void beginMotion(int motiontype);

        void ctrlStateMachine();

    };


}


#endif // !__ROBOT_H__

