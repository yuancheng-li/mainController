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
    /* ���������ˣ���¼�������� */
    robotPtr->setPowerState(true, ec);
    auto rtCon = robotPtr->getRtMotionController().lock();
    robotPtr->startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m });
    
    /* ��ʼ������״̬��״̬ */
    m_SM.state = SM_STATE::STOPPED;
    m_SM.action = SM_ACTION::ENABLE_MOTORS;
    /* �����˶������߳� ��������״̬��ctrlStateMachine */
    cout << "Start Control Thread" << endl;
    std::thread ctrlThread([this] {
        while (!m_stopCtrl) {
            ctrlStateMachine();
		}
	});

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

}


