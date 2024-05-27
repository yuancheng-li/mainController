#include "mrobot.h"

using namespace mrobot;
using namespace rokae;

namespace preDefines {
    // ******   ��קλ�ˡ�����λ�ˡ����λ�ˡ�����λ�ˡ��궨λ��   ******
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

/**
 * @brief ��ӡ�˶�ִ����Ϣ
 */
void printInfo(const rokae::EventInfo& info) {
    using namespace rokae::EventInfoKey::MoveExecution;
    print(std::cout, "[�˶�ִ����Ϣ] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "�����: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES" : "NO", std::any_cast<error_code>(info.at(Error)),
        std::any_cast<std::string>(info.at(Remark)));
}

void robotArm::startCtrl_n(std::error_code ec)
{
    ostream& os = std::cout;
    /* �ϵ� */
    robotPtr->setPowerState(true, ec);
    if (ec) {
		print(cerr, "ʹ�ܵ��ʧ��", ec.message());
	}

    robotPtr->setEventWatcher(Event::moveExecution, printInfo, ec);
    if (ec) {
		print(cerr, "�����¼�����ʧ��", ec.message());
	}

    // �����̣߳���ȡ���룬���в�ͬ�Ĳ���
    std::thread input([&] {
        int c{};
        print(os, "[p]��ͣ [c]���� [q]�˳� [d]�϶� [a]������λ�˶� [b]���ݲɼ���λ");
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

                // �������ݲɼ�λ��
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

                // ������ݲɼ�λ��
                for (int i = 0; i < dataCollectPose.size(); i++) {
                    cout << "dataCollectPose[" << i << "]: " << endl;
                    for (int j = 0; j < 6; j++) {
                        cout << dataCollectPose[i][j] << " ";
                    }
                    cout << endl;
                }
                for (auto& dataCollectPose : dataCollectPose) {
					// ����MoveLָ��(ֱ���˶������ݲɼ�λ��
                    MoveLCommand moveL(dataCollectPose);
                    robotPtr->executeCommand({ moveL }, ec);
                    print(std::cerr, ec);
                    WaitRobot();
                }
                cout << "���ݲɼ����" << endl;
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
                print(os, "�˶�����קλ��");
                robotPtr->executeCommand({ preDefines::moveDrag }, ec);
                print(std::cerr, ec); break;

                robotPtr->setOperateMode(rokae::OperateMode::manual, ec);
                robotPtr->setPowerState(false, ec); // �Զ�ģʽ���ϵ�
                robotPtr->enableDrag(DragParameter::jointSpace, DragParameter::freely, ec);
                std::cout << "���϶����: " << ec.message() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2)); //�ȴ��л�����ģʽ
                auto power = robotPtr->powerState(ec);
                std::cout << "���µ�״̬: " << static_cast<int>(power) << std::endl; //���϶���Ӧ�Զ��ϵ�
                std::cout << "���س����ر��϶�" << std::endl;
                array<double, 42> jacob_r;
                array<double, 7> jpos_r;

                auto model = robotPtr->model();
                while (getchar() != '\n') {
                    jpos_r = robotPtr->jointPos(ec);
                    jacob_r = model.jacobian(jpos_r, SegmentFrame::flange);
                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> JacMat(jacob_r.data(), 7, 6);
                    std::cout << "��ǰ���ݶ�ָ�꣺ " << m_manipulability.calcMani(JacMat) << endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                robotPtr->disableDrag(ec);
                std::this_thread::sleep_for(std::chrono::seconds(2)); //�ȴ��л�����ģʽ 
                break;
            }     
            default: break;
            }
        }
        });
    input.join(); // ����ʽ�ȴ������߳̽���
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


