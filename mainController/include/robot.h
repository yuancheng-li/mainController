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
        POWER_ON = 0x01,
        POWER_OFF = 0x02,
        BEGIN_MOTION = 0x03,
        STOP_MOTION = 0x04
    };

    enum MOTION_TYPE
    {
        CARTESIAN_S_LINE = 0x01,
        JOINT_S_LINE = 0x02,
        CARTESIAN_MODE = 0x03,
        JOINT_MODE = 0x04,
        
    };


    class robotArm
    {
    public:
        robotArm();
        ~robotArm();

#ifdef xMateErPro
        std::unique_ptr<rokae::xMateErProRobot> robotPtr;
#endif
        std::shared_ptr<rokae::RtMotionControlCobot<m_DoF>> rtCon;
        bool m_stopCtrl;
        float m_TStep;

        struct StateMachine
        {
            int action;
            int state;
            int motiontype;
        } m_SM;

        struct CartisianSLine
        {
            // ���Ʋ���
            bool onoff;
            bool firsttime;
            uint16_t substep;

            // ����
            std::array<double, 16> init_pos{}, end_pos{};
            std::array<double, 7> jntPos{}, delta{};
            Eigen::Quaterniond rot_cur;
            Eigen::Matrix3d mat_cur;
            double delta_s;
		} m_CartSLine;

        struct JointSLine
        {
            // ���Ʋ���
            bool onoff;
            bool firsttime;
            uint16_t substep;

            // ����
            std::array<double, 16> init_pos{}, end_pos{};
            std::array<double, 7> jntPos{}, delta{};
            Eigen::Quaterniond rot_cur;
            Eigen::Matrix3d mat_cur;
            double delta_s;
        } m_JntSLine;
        
        void connect(const std::string& robotIP, const std::string& controllerIP);
        void init();

        /* �����˶������߳̿�������״̬�� */
        void startCtrl(std::error_code ec);

        /* ��ʵʱ�˶����� */
        void startCtrl_n(std::error_code ec);

        /* ֹͣ�˶������̵߳�״̬��ѭ�� */
        void stopCtrl();

        /* ֹͣ��ʵʱ���� */
        void stopCtrl_n(std::error_code ec);

        /* ��ʼ�˶�,�����˶����� */
        void beginMotion(int motiontype);

        /* ����״̬�� */
        void ctrlStateMachine();

        /* ������״̬��� */
        void WaitRobot();

    };


}


#endif // !__ROBOT_H__

