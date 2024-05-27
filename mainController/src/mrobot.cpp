#include "mrobot.h"

using namespace mrobot;
using namespace rokae;

namespace preDefines {
    // ******   拖拽位姿、工作位姿、打包位姿、搬运位姿、标定位姿   ******
    // xMateEr7 Pro
    JointPosition q_drag = { 0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0 };
    JointPosition q_work = { 0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, M_PI / 2 };
    JointPosition q_pack = { 0 * M_PI / 180, 105 * M_PI / 180, 0 * M_PI / 180, 90 * M_PI / 180, 0 * M_PI / 180, -15 * M_PI / 180, 0 * M_PI / 180 };
    JointPosition q_lift = { 90 * M_PI / 180,0 * M_PI / 180,90 * M_PI / 180,90 * M_PI / 180,0 * M_PI / 180,90 * M_PI / 180, 0 * M_PI / 180 };
    JointPosition q_calib = { 1.45 * M_PI / 180,73 * M_PI / 180,3.2 * M_PI / 180,72 * M_PI / 180,-2 * M_PI / 180, 70.2 * M_PI / 180, 95.3 * M_PI / 180 };
    MoveAbsJCommand moveDrag(q_drag);
    MoveAbsJCommand moveWork(q_work);
    MoveAbsJCommand movePack(q_pack);
    MoveAbsJCommand moveLift(q_lift);
    MoveAbsJCommand moveCalib(q_calib);
}

robotArm::robotArm() 
{
    m_TStep = 0.001f;
    m_stopCtrl = false;
    m_SM.state = SM_STATE::STOPPED;

    //m_poseGenerate = poseGenerate(this);
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

/**
 * @brief 打印运动执行信息
 */
void printInfo(const rokae::EventInfo& info) {
    using namespace rokae::EventInfoKey::MoveExecution;
    print(std::cout, "[运动执行信息] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "已完成: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES" : "NO", std::any_cast<error_code>(info.at(Error)),
        std::any_cast<std::string>(info.at(Remark)));
}

void robotArm::startCtrl_n(std::error_code ec)
{
    ostream& os = std::cout;
    /* 上电 */
    robotPtr->setPowerState(true, ec);
    if (ec) {
		print(cerr, "使能电机失败", ec.message());
	}

    robotPtr->setEventWatcher(Event::moveExecution, printInfo, ec);
    if (ec) {
		print(cerr, "设置事件监听失败", ec.message());
	}

    // 开启线程，读取输入，运行不同的操作
    std::thread input([&] {
        int c{};
        print(os, "[p]暂停 [c]继续 [q]退出 [d]拖动 [a]工作点位运动 [b]数据采集点位");
        while (c != 'q') {
            c = getchar();
            switch (c) {
            case 'a':
                robotPtr->executeCommand({ preDefines::moveDrag, preDefines::moveWork,
                    preDefines::moveDrag, preDefines::movePack,
                    preDefines::moveDrag, preDefines::moveLift,
                    preDefines::moveDrag, preDefines::moveCalib }, ec);
                print(std::cerr, ec);
                WaitRobot();
                break;
            case 'b': {
                auto& [circCenter, endPos, anglesX, anglesY, dataCollectPose] = m_pG.m_dC;

                // 生成数据采集位姿
                anglesX.resize(7);
                anglesX << -45, -30, -15, 0, 15, 30, 45;
                anglesY.resize(7);
                anglesY << -45, -30, -15, 0, 15, 30, 45;

                //cout << anglesX << endl;
                robotPtr->executeCommand({ preDefines::moveDrag, preDefines::moveCalib }, ec);
                print(std::cerr, ec);
                WaitRobot();
                endPos = robotPtr->posture(rokae::CoordinateType::flangeInBase, ec);

                //endPos = { 1.0, 0.0, 2.0, 90.0 / 180 * M_PI, 0.0, 90.0 / 180 * M_PI };
                circCenter = { 2.0, 0.0, 2.0 };

                dataCollectPose = m_pG.generateDataCollectPose(circCenter, endPos, anglesX, anglesY);

                // 输出数据采集位姿
                for (int i = 0; i < dataCollectPose.size(); i++) {
                    cout << "dataCollectPose[" << i << "]: " << endl;
                    for (int j = 0; j < 6; j++) {
                        cout << dataCollectPose[i][j] << " ";
                    }
                    cout << endl;
                }
                for (auto& dataCollectPose : dataCollectPose) {
					// 生成MoveL指令(直线运动到数据采集位姿
                    MoveLCommand moveL(dataCollectPose);
                    robotPtr->executeCommand({ moveL }, ec);
                    print(std::cerr, ec);
                    WaitRobot();
                }
                cout << "数据采集完成" << endl;
                break;
            }
            case 'p':
                robotPtr->stop(ec);
                print(std::cerr, ec); 
                break;
            case 'c':
                robotPtr->moveStart(ec);
                print(std::cerr, ec); 
                break;
            case 'd':
            {
                print(os, "运动到拖拽位置");
                robotPtr->executeCommand({ preDefines::moveDrag }, ec);
                print(std::cerr, ec); break;

                robotPtr->setOperateMode(rokae::OperateMode::manual, ec);
                robotPtr->setPowerState(false, ec); // 自动模式下上电
                robotPtr->enableDrag(DragParameter::jointSpace, DragParameter::freely, ec);
                std::cout << "打开拖动结果: " << ec.message() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
                auto power = robotPtr->powerState(ec);
                std::cout << "上下电状态: " << static_cast<int>(power) << std::endl; //打开拖动后应自动上电
                std::cout << "按回车键关闭拖动" << std::endl;
                array<double, 42> jacob_r;
                array<double, 7> jpos_r;

                auto model = robotPtr->model();
                while (getchar() != '\n') {
                    jpos_r = robotPtr->jointPos(ec);
                    jacob_r = model.jacobian(jpos_r, SegmentFrame::flange);
                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> JacMat(jacob_r.data(), 7, 6);
                    std::cout << "当前操纵度指标： " << m_manipulability.calcMani(JacMat) << endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                robotPtr->disableDrag(ec);
                std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式 
                break;
            }     
            default: break;
            }
        }
        });
    input.join(); // 阻塞式等待输入线程结束
    robotPtr->moveReset(ec);

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


