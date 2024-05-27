﻿/**
 * @file model.h
 * @brief xMateModel模型库
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_MODEL_H
#define ROKAEAPI_MODEL_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "rokae/data_types.h"

namespace rokae {

// forward declarations
 class BaseRobot;
 class XService;
 struct Info;

 /**
  * @class BaseModel
  * @brief 模型通用类
  */
 class XCORE_API BaseModel : public Base<BaseModel>{
  public:

  /// @cond DO_NOT_DOCUMENT
   explicit BaseModel(std::shared_ptr<XService> rpc);
   virtual ~BaseModel();
  /// @endcond

   /**
    * @brief 用户定义的基坐标系, 相对于世界坐标系
    * @param[out] ec 错误码
    * @return 数组, 长度: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
    * transformation and \f$ \mathbb{R}^{3 \times 1} \f$ rotation \f$ [x, y, z, a, b, c]^T \f$.
    */
   [[deprecated("Use BaseRobot::baseFrame() instead")]]
   std::array<double, 6> baseFrame(error_code &ec) const noexcept;

   /**
    * @brief 查询当前工具工件组信息
    * @note 此工具工件组仅为SDK运动控制使用, 不与RL工程相关.
    * @param[out] ec 错误码
    */
   [[deprecated("Use BaseRobot::toolset() instead")]]
   Toolset toolset(std::error_code &ec) const noexcept;

   /**
    * @brief 设置工具工件组信息
    * @note 此工具工件组仅为SDK运动控制使用, 不与RL工程相关.
    *       除此接口外, 如果通过RobotAssist更改默认工具工件(右上角的选项), 该工具工件组也会相应更改.
    * @param[in] toolset 工具工件组信息
    * @param[out] ec 错误码
    */
   [[deprecated("Use BaseRobot::setToolset() instead")]]
   void setToolset(const Toolset& toolset, error_code &ec) noexcept;

   /**
    * @brief 使用已创建的工具和工件，设置工具工件组信息
    * @note 设置前提: 已加载一个RL工程，且创建了工具和工件。否则，只能设置为默认的工具工件，即"tool0"和"wobj0"。
    * 一组工具工件无法同时为手持或外部；如果有冲突，以工具的位置为准，例如工具工件同时为手持，不会返回错误，但是工件的坐标系变成了外部
    * @param[in] toolName 工具名称
    * @param[in] wobjName 工件名称
    * @param[out] ec 错误码
    * @return 设置后的工具工件组信息。当发生错误设置失败时，返回Toolset类型初始化默认值0
    */
   [[deprecated("Use BaseRobot::setToolset() instead")]]
   Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept;

  XCORESDK_DECLARE_IMPL
 };

/**
 * @class Model_T
 * @brief 模型模板类
 * @tparam DoF 轴数
 */
 template<unsigned short DoF>
 class XCORE_API Model_T : public BaseModel {

  public:
   using BaseModel::BaseModel;

   /**
    * @brief 根据位姿计算逆解
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double, DoF> calcIk(CartesianPosition posture, error_code &ec) noexcept;

   /**
    * @brief 根据轴角度计算正解
    * @param[in] joints 轴角度, 单位: 弧度
    * @param[out] ec 错误码
    * @return 机器人末端位姿，相对于外部参考坐标系
    */
   CartesianPosition calcFk(const std::array<double, DoF> &joints, error_code &ec) noexcept;

 };

 /**
  * @enum SegmentFrame
  * @brief 连杆标号
  */
 enum class SegmentFrame : unsigned {
   joint1 = 1, joint2 = 2, joint3 = 3, joint4 = 4, joint5 = 5,
   joint6 = 6, joint7 = 7, flange = 8, endEffector = 9, stiffness = 10 };

 /**
  * @enum TorqueType
  * @brief 力矩类型
  */
 enum class TorqueType {
   full,     ///< 关节力矩，由动力学模型计算得到
   inertia,  ///< 惯性力
   coriolis, ///< 科氏力
   friction, ///< 摩擦力
   gravity   ///< 重力
 };
#define XMATEMODEL_LIB_SUPPORTED
#ifdef XMATEMODEL_LIB_SUPPORTED

 template <unsigned short DoF>
 class XCORE_API xMateModel : public Model_T<DoF> {

  public:
   /**
    * @brief Create an xMateModel instance
    * @throw ExecutionException 加载模型失败
    */
   explicit xMateModel(std::shared_ptr<XService> rpc, const Info& info);
   ~xMateModel();

   /**
    * @brief 设置负载参数，只在计算时使用，并不将参数传给机器人控制器，设置后动力学计算结果相应改变
    * @param[in] mass 质量
    * @param[in] cog 质心, 单位: m
    * @param[in] inertia 惯量
    * @see Load
    */
   void setLoad(double mass, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   /**
    * @brief 设置TCP工具，只在计算时使用，并不将参数传给机器人控制器，设置TCP后，正逆解结果和输入参数相应改变
    * @param[in] f_t_ee 末端执行器相对于法兰的位姿
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿
    */
   void setTcpCoor(const std::array<double, 16> &f_t_ee, const std::array<double, 16> &ee_t_k);

   /**
    * @brief 获取笛卡尔空间位置
    * @param[in] jntPos 需要计算笛卡尔位姿的关节角度
    * @param[in] nr 指定坐标系, 缺省值为flange
    * @return 向量化4x4位姿矩阵，行优先.
    */
   std::array<double, 16> getCartPose(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取笛卡尔空间速度
    * @param[in] jntPos 需要计算笛卡尔空间速度的关节角度
    * @param[in] jntVel 需要计算笛卡尔空间速度的关节角速度
    * @param[in] nr 指定坐标系, 缺省值为flange
    * @return 计算结果
    */
   std::array<double, 6> getCartVel(const std::array<double, DoF> &jntPos, const std::array<double, DoF> &jntVel,
                                    SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取笛卡尔空间加速度
    * @param[in] jntPos 需要计算笛卡尔空间速度的关节角度
    * @param[in] jntVel 需要计算笛卡尔空间速度的关节角速度
    * @param[in] jntAcc 需要计算笛卡尔空间速度的关节角加速度
    * @param[in] nr 指定坐标系
    * @return 计算结果
    */
   std::array<double, 6> getCartAcc(const std::array<double, DoF> &jntPos,
                                    const std::array<double, DoF> &jntVel,
                                    const std::array<double, DoF> &jntAcc,
                                    SegmentFrame nr = SegmentFrame::flange );

   /**
    * @brief 逆解获得关节空间位置。一个位姿可能对应多个关节角度，jntPos的选取原则是选取一个与jntInit最近的解。
    * @param[in] cartPos 法兰笛卡尔空间位姿
    * @param[in] elbow 臂角
    * @param[in] jntInit 初始关节角度
    * @param[out] jntPos 关节空间位置
    * @return 计算逆解结果 -
    *    1) -1, -2, -3: 无解，原因是cartPos超出机器人工作空间;
    *    2) -4, -5: jntPos与jntInit相差较大，一般认为jntInit代表机器人当前位置，jntPos与jntInit之差可以等效为电机转速。
    *               若超过机器人轴额定转速，则返回-4或-5;
    *    3) -6, -7: jntPos超过软限位;
    *    4)	-8: 机器人奇异；
    */
    int getJointPos(const std::array<double, 16> &cartPos,
                    double elbow,
                    const std::array<double, DoF> &jntInit,
                    std::array<double, DoF> &jntPos);

   /**
    * @brief 逆解获得关节空间速度
    * @param[in] cartVel 法兰笛卡尔空间速度
    * @param[in] jntPos 此时关节角度
    * @return 计算结果
    */
   std::array<double, DoF> getJointVel(const std::array<double, 6> &cartVel, const std::array<double, DoF> &jntPos);

   /**
    * @brief 逆解获得关节空间加速度
    * @param[in] cartAcc 法兰笛卡尔空间加速度
    * @param[in] jntPos 此时关节角度
    * @param[in] jntVel 此时关节角速度
    * @return 计算结果
    */
   std::array<double, DoF> getJointAcc(const std::array<double, 6> &cartAcc,
                                       const std::array<double, DoF> &jntPos,
                                       const std::array<double, DoF> &jntVel);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵, 行优先
    * @param[in] jntPos 关节角度.
    * @param[in] nr 指定坐标系
    * @return 计算结果, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵, 行优先
    * @param[in] jntPos 关节角度.
    * @param[in] f_t_ee 末端执行器相对于法兰坐标系的位姿.
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿.
    * @param[in] nr 指定坐标系
    * @return 计算结果, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos,
                                      const std::array<double, 16> &f_t_ee,
                                      const std::array<double, 16> &ee_t_k,
                                      SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 由模型计算关节力矩
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[in] torque_type 指定力矩类型
    * @return 计算结果，单位: Nm
    */
   std::array<double, DoF> getTorque(const std::array<double, DoF> &jntPos,
                                     const std::array<double, DoF> &jntVel,
                                     const std::array<double, DoF> &jntAcc,
                                     TorqueType torque_type);

   /**
    * @brief 由模型计算关节力矩，计算结果单位：Nm。如有负载，先通过setLoad()设置负载参数。
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[out] trq_full 总关节力矩
    * @param[out] trq_inertia 离心力
    * @param[out] trq_coriolis 科氏力
    * @param[out] trq_friction 关节摩擦力
    * @param[out] trq_gravity 重力矩
    */
   void getTorqueWithFriction(const std::array<double, DoF> &jntPos,
                              const std::array<double, DoF> &jntVel,
                              const std::array<double, DoF> &jntAcc,
                              std::array<double, DoF> &trq_full,
                              std::array<double, DoF> &trq_inertia,
                              std::array<double, DoF> &trq_coriolis,
                              std::array<double, DoF> &trq_friction,
                              std::array<double, DoF> &trq_gravity);

   /**
    * @brief 由模型计算无摩擦力的关节力矩, 计算结果单位: Nm。如有负载，先通过setLoad()设置负载参数。
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[out] trq_full 总关节力矩
    * @param[out] trq_inertia 离心力
    * @param[out] trq_coriolis 科氏力
    * @param[out] trq_gravity 重力矩
    */
   void getTorqueNoFriction(const std::array<double, DoF> &jntPos,
                            const std::array<double, DoF> &jntVel,
                            const std::array<double, DoF> &jntAcc,
                            std::array<double, DoF> &trq_full,
                            std::array<double, DoF> &trq_inertia,
                            std::array<double, DoF> &trq_coriolis,
                            std::array<double, DoF> &trq_gravity);

  XCORESDK_DECLARE_IMPLD
 };

 template <unsigned short DoF>
 using XMateModel = xMateModel<DoF>;

#endif // #ifdef XMATEMODEL_LIB_SUPPORTED

}  // namespace rokae

#endif // ROKAEAPI_MODEL_H
