#include "robot.h"
using namespace robot;
using namespace rokae;


robotArm::robotArm()
{
    m_TStep = 0.001f;
    m_stopCtrl = false;
    m_SM.state = SM_STATE::STOPPED;
}

robotArm::~robotArm()
{

}

void robotArm::connect(const std::string& robotIP, const std::string& controllerIP)
{
    try {
        robotPtr = std::make_unique<rokae::xMateErProRobot>(robotIP, controllerIP);
    }
    catch (const rokae::NetworkException& e) {
        std::cout << "Network Exception: " << e.what() << std::endl;
        // ���������쳣
    }
    catch (const rokae::ExecutionException& e) {
        std::cout << "Execution Exception: " << e.what() << std::endl;
        // ����ִ���쳣
    }
    catch (const std::exception& e) {
        std::cout << "Standard Exception: " << e.what() << std::endl;
        // ����������׼�쳣
    }
    catch (...) {
        std::cout << "Unknown Exception" << std::endl;
        // ����δ֪�쳣
    }
    
    /* ʹ��robotInfo��ȡ����ӡ��������Ϣ */
    std::error_code ec;
    auto robotinfo = robotPtr->robotInfo(ec);
    /* �����Ϣ */
    std::cout << "Robot Info: " << std::endl;
    std::cout << "�������汾��: " << robotinfo.version << std::endl;
    std::cout << "����: " << robotinfo.type << std::endl;
    std::cout << "xCore-SDK�汾: " << robotPtr->sdkVersion() << std::endl;

}


void robotArm::init()
{
    // ���ò���ģʽ������ģʽ��
    std::error_code ec;
    robotPtr->setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
        print(std::cerr, "���ò���ģʽʧ��: " + ec.message());
        // ������Ҫ���ػ�ִ�������������߼�
    }
}


void robotArm::startCtrl(std::error_code ec)
{
    /* �����˶������߳� ��������״̬��ctrlStateMachine */
    cout << "Start Control Thread" << endl;
    std::thread ctrlThread([this] {
        while (!m_stopCtrl) {
            ctrlStateMachine();
		}
	});

    /* ��ʼ������״̬��״̬ */
    rtCon = robotPtr->getRtMotionController().lock();
    m_SM.state = SM_STATE::STOPPED;
    m_SM.action = SM_ACTION::POWER_ON;
}

void robotArm::WaitRobot() {
    bool running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robotPtr->operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) {
            running = false;
        }
    }
}

void robotArm::startCtrl_n(std::error_code ec)
{
    ostream& os = std::cout;
    /* ���ù��ߡ���������ϵ */
    auto model = robotPtr->model();
    // Ĭ�ϵĹ��߹��� tool0, wobj0
    auto currentToolset = robotPtr->setToolset("tool0", "wobj0", ec);
    print(os, "��ǰ���߹�����", currentToolset);

    MoveAbsJCommand moveAbs1({ 0, 0, 0, 0, 0, 0, 0 });
    MoveAbsJCommand moveAbs2({ 0, M_PI / 6, -M_PI / 2, 0, -M_PI / 3, 0, 0 });
    robotPtr->executeCommand({ moveAbs1, moveAbs2 }, ec);
    /*MoveLCommand movel1({ 0.563, 0, 0.432, M_PI, 0, M_PI }, 200, 100);
    MoveLCommand movel2({ 0.33467, -0.095, 0.51, M_PI, 0, M_PI }, 200, 100);
    robotPtr->executeCommand({ movel1, movel2 }, ec);*/
    if (ec) {
        print(cerr, "ִ��ǰ����", ec.message());
    }
    else {
        WaitRobot();
        ec = robotPtr->lastErrorCode();
        if (ec) {
            print(cerr, "�˶�ִ���д���:", robotPtr->lastErrorCode().message());
        }
    }
}

void robotArm::stopCtrl()
{
    m_stopCtrl = true;
}

void robotArm::stopCtrl_n(std::error_code ec)
{
    robotPtr->setPowerState(false, ec);
    robotPtr->disconnectFromRobot(ec);
}

void robotArm::beginMotion(int motiontype)
{
	/* ����ֹͣ״̬����ָ�� */
	if (m_SM.state == SM_STATE::STOPPED) {
		m_SM.action = SM_ACTION::BEGIN_MOTION;
		m_SM.motiontype = motiontype;
		m_SM.state = SM_STATE::RUNNING;
	}

}

void robotArm::ctrlStateMachine()
{
    switch (m_SM.state) {
	case SM_STATE::STOPPED:
        if (m_SM.action == SM_ACTION::POWER_ON) {

			/* ʹ�ܵ�� */
			std::error_code ec;
            robotPtr->setPowerState(true, ec);
            if (ec) {
                print(std::cerr, "ʹ�ܵ��ʧ��: " + ec.message());
            }

            /* ��¼�������� */
            robotPtr->startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m });
			m_SM.action = SM_ACTION::NONE;
		}

        if (m_SM.action == SM_ACTION::BEGIN_MOTION) {
            /* �����˶�����ִ�в�ͬ���˶� */
			switch (m_SM.motiontype) 
            {
                case MOTION_TYPE::CARTESIAN_S_LINE:
                {
					m_CartSLine.onoff = true;
                    m_CartSLine.firsttime = true;
                    m_CartSLine.substep = 0;  
					break;
				}
                
            }
            m_SM.action = SM_ACTION::NONE;
            m_SM.state = SM_STATE::RUNNING;
        }
		break;
	case SM_STATE::RUNNING:
        if (m_SM.action == SM_ACTION::STOP_MOTION)
        {
            /* �����˶����������� */
		}
        if (m_CartSLine.onoff) {
            if (m_CartSLine.substep == 0) {
				/* ��ʼ������ */
				m_CartSLine.substep = 1;
				m_CartSLine.firsttime = false;
            }
            else if (m_CartSLine.substep == 1) {

            }
            else {
				m_CartSLine.onoff = false;
                m_CartSLine.substep = 0;
			}
        }
        else {
            /* ���˶� */
        }
		break;
	default:
		break;
	}
}


