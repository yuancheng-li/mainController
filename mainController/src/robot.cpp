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

void robotArm::stopCtrl()
{
    m_stopCtrl = true;
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


