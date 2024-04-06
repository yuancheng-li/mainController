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
    /* 启动机器人，记录数据类型 */
    robotPtr->setPowerState(true, ec);
    auto rtCon = robotPtr->getRtMotionController().lock();
    robotPtr->startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m });
    
    /* 初始化核心状态机状态 */
    m_SM.state = SM_STATE::STOPPED;
    m_SM.action = SM_ACTION::ENABLE_MOTORS;
    /* 创建运动控制线程 开启核心状态机ctrlStateMachine */
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
	/* 仅在停止状态接收指令 */
	if (m_SM.state == SM_STATE::STOPPED) {
		m_SM.action = SM_ACTION::BEGIN_MOTION;
		m_SM.motiontype = motiontype;
		m_SM.state = SM_STATE::RUNNING;
	}

}

void robotArm::ctrlStateMachine()
{

}


