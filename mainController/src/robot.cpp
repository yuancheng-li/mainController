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
        // 处理网络异常
    }
    catch (const rokae::ExecutionException& e) {
        std::cout << "Execution Exception: " << e.what() << std::endl;
        // 处理执行异常
    }
    catch (const std::exception& e) {
        std::cout << "Standard Exception: " << e.what() << std::endl;
        // 处理其他标准异常
    }
    catch (...) {
        std::cout << "Unknown Exception" << std::endl;
        // 处理未知异常
    }
    
    /* 使用robotInfo获取并打印机器人信息 */
    std::error_code ec;
    auto robotinfo = robotPtr->robotInfo(ec);
    /* 输出信息 */
    std::cout << "Robot Info: " << std::endl;
    std::cout << "控制器版本号: " << robotinfo.version << std::endl;
    std::cout << "机型: " << robotinfo.type << std::endl;
    std::cout << "xCore-SDK版本: " << robotPtr->sdkVersion() << std::endl;

}


void robotArm::init()
{
    // 设置操作模式、控制模式等
    std::error_code ec;
    robotPtr->setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
        print(std::cerr, "设置操作模式失败: " + ec.message());
        // 可能需要返回或执行其他错误处理逻辑
    }
}


void robotArm::startCtrl(std::error_code ec)
{
    /* 创建运动控制线程 开启核心状态机ctrlStateMachine */
    cout << "Start Control Thread" << endl;
    std::thread ctrlThread([this] {
        while (!m_stopCtrl) {
            ctrlStateMachine();
		}
	});

    /* 初始化核心状态机状态 */
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
	/* 仅在停止状态接收指令 */
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

			/* 使能电机 */
			std::error_code ec;
            robotPtr->setPowerState(true, ec);
            if (ec) {
                print(std::cerr, "使能电机失败: " + ec.message());
            }

            /* 记录数据类型 */
            robotPtr->startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m });
			m_SM.action = SM_ACTION::NONE;
		}

        if (m_SM.action == SM_ACTION::BEGIN_MOTION) {
            /* 根据运动类型执行不同的运动 */
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
            /* 结束运动，保存数据 */
		}
        if (m_CartSLine.onoff) {
            if (m_CartSLine.substep == 0) {
				/* 初始化步骤 */
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
            /* 无运动 */
        }
		break;
	default:
		break;
	}


}


