#include "quadrotor_simulator/Quadrotor.h"
#include "ode/boost/numeric/odeint.hpp"
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
namespace odeint = boost::numeric::odeint;


/*在这里本文仿真了一个四旋翼的模型，具体的模型参数已经在代码中呈现并进行备注，
你无需也不能进行修改四旋翼的模型参数，你所要做的就是阅读相关代码，并结合已学习的知识，补充四旋翼的dynamic代码。*/

/*state表示的是四旋翼的状态的一个矩阵，x表示四旋翼的位置，v表示速度，R表示姿态，omega表示角速度，motor_rpm表示电机转速，四旋翼的状态由以上几个参数进行表示*/

namespace QuadrotorSimulator
{

Quadrotor::Quadrotor(void)    //模型的初始化
{
  alpha0     = 48; // degree    //螺旋桨的初始攻角 
  g_         = 9.81;            //重力加速度                      
  mass_      = 0.98; // 0.5;    //四旋翼飞行器的质量
  double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;    //惯性矩相关参数，分别表示绕x、y、z轴的惯性矩
  prop_radius_ = 0.062;          //螺旋桨半径
  J_           = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();   // 惯性矩阵

  kf_ = 8.98132e-9;                            //螺旋桨推力系数
  // km_ = 2.5e-9; // from Nate
  // km = (Cq/Ct)*Dia*kf
  // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
  km_ = 0.07 * (3 * prop_radius_) * kf_;            //螺旋桨力矩系数，用于计算螺旋桨产生的力矩

  arm_length_          = 0.26;             // 螺旋桨臂长
  motor_time_constant_ = 1.0 / 30;         // 电机时间常数
  min_rpm_             = 1200;             // 电机最小转速
  max_rpm_             = 35000;            // 电机最大转速

  state_.x = Eigen::Vector3d::Zero();        //初始化
  // state_.x << 40.0, -60.0, 10.0;
  state_.v         = Eigen::Vector3d::Zero();
  state_.R         = Eigen::Matrix3d::Identity();
  state_.omega     = Eigen::Vector3d::Zero();
  state_.motor_rpm = Eigen::Array4d::Zero();

  external_force_.setZero();

  updateInternalState();

  input_ = Eigen::Array4d::Zero();
}


void Quadrotor::updateInternalState(void)    //在内部进行状态存储和传递的变量
{
  for (int i = 0; i < 3; i++)
  {
    internal_state_[0 + i]  = state_.x(i);
    internal_state_[3 + i]  = state_.v(i);
    internal_state_[6 + i]  = state_.R(i, 0);
    internal_state_[9 + i]  = state_.R(i, 1);
    internal_state_[12 + i] = state_.R(i, 2);
    internal_state_[15 + i] = state_.omega(i);
  }

  internal_state_[18] = state_.motor_rpm(0);
  internal_state_[19] = state_.motor_rpm(1);
  internal_state_[20] = state_.motor_rpm(2);
  internal_state_[21] = state_.motor_rpm(3);
}

void Quadrotor::step(double dt) 
{
  /*step函数主要用于更新四旋翼飞行器的状态。它基于数值积分方法，根据当前状态和物理模型，
  在给定的时间步长dt内计算并更新飞行器的状态。这使得四旋翼飞行器的模拟能够随着时间逐步推进，
  模拟其在实际环境中的动态行为。*/

  auto save = internal_state_;

  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);   
  //调用下面的void Quadrotor::operator()(const Quadrotor::InternalState& x,Quadrotor::InternalState& dxdt, const double /* t */)数值积分操作
  // operator(state_, internal_state_, dt);
  //所以你要修改的地方其实就是operator函数

  for (int i = 0; i < 22; ++i)     //异常处理
  {
    if (std::isnan(internal_state_[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    state_.x(i) = internal_state_[0 + i];
    state_.v(i) = internal_state_[3 + i];
    state_.R(i, 0) = internal_state_[6 + i];
    state_.R(i, 1) = internal_state_[9 + i];
    state_.R(i, 2) = internal_state_[12 + i];
    state_.omega(i) = internal_state_[15 + i];
  }
  state_.motor_rpm(0) = internal_state_[18];
  state_.motor_rpm(1) = internal_state_[19];
  state_.motor_rpm(2) = internal_state_[20];
  state_.motor_rpm(3) = internal_state_[21];
  // std::cout<<state_.motor_rpm;
  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = state_.R * P.inverse();
  state_.R                      = R;

  // 不要撞到地面上
  if (state_.x(2) < 0.0 && state_.v(2) < 0)   
  {
    state_.x(2) = 0;
    state_.v(2) = 0;
  }
  updateInternalState();
}

void Quadrotor::operator()(const Quadrotor::InternalState& x, Quadrotor::InternalState& dxdt, const double /* t */)
{
  State cur_state;
  for (int i = 0; i < 3; i++)    //状态提取
  {
    cur_state.x(i) = x[0 + i];
    cur_state.v(i) = x[3 + i];
    cur_state.R(i, 0) = x[6 + i];
    cur_state.R(i, 1) = x[9 + i];
    cur_state.R(i, 2) = x[12 + i];
    cur_state.omega(i) = x[15 + i];
  }
  for (int i = 0; i < 4; i++)
  {
    cur_state.motor_rpm(i) = x[18 + i];
    // std::cout << cur_state.motor_rpm;
  }


  // 姿态矩阵处理（重新正交化）
  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  //中间变量定义和计算,定义了一系列用于后续计算的中间变量。
  Eigen::Vector3d x_dot, v_dot, omega_dot;  //分别用于存储位置、速度和角速度的导数
  Eigen::Matrix3d R_dot;                    //用于存储姿态矩阵的导数
  Eigen::Array4d  motor_rpm_dot;            //用于存储电机转速的导数
  Eigen::Vector3d vnorm;                    //用于存储速度的单位向量
  Eigen::Array4d  motor_rpm_sq;             //用于存储电机转速的平方
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());    //角速度的反对称矩阵

  omega_vee(2, 1) = cur_state.omega(0);   //角速度的反对称矩阵
  omega_vee(1, 2) = -cur_state.omega(0);
  omega_vee(0, 2) = cur_state.omega(1);
  omega_vee(2, 0) = -cur_state.omega(1);
  omega_vee(1, 0) = cur_state.omega(2);
  omega_vee(0, 1) = -cur_state.omega(2);

  motor_rpm_sq = cur_state.motor_rpm.array().square();    //于存储电机转速的平方

  //! @todo implement
  Eigen::Array4d blade_linear_velocity;
  Eigen::Array4d motor_linear_velocity;
  Eigen::Array4d AOA;   //攻角的计算
  blade_linear_velocity = 0.104719755 * cur_state.motor_rpm.array() * prop_radius_;
  for (int i = 0; i < 4; ++i){
    AOA[i]   = alpha0 - atan2(motor_linear_velocity[i], blade_linear_velocity[i]) * 180 / 3.14159265;
  }


  // double totalF = kf_ * motor_rpm_sq.sum();
  double thrust = kf_ * motor_rpm_sq.sum();  //总推力

  Eigen::Vector3d moments;  //力矩
  moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
  moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
  moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
                      motor_rpm_sq(3));

  double resistance = 0.15 *                                        // C
                      3.14159265 * (arm_length_) * (arm_length_) * // S
                      cur_state.v.norm() * cur_state.v.norm();


  vnorm = cur_state.v;
  if (vnorm.norm() != 0)
  {
    vnorm.normalize();
  }

  x_dot = cur_state.v;
  //请在这里补充完四旋翼飞机的动力学模型，提示：v_dot应该与重力，总推力，外力和空气阻力相关
  // 获取飞行器的物理参数
  double mass = mass_;  // 质量
  double gravity = g_;  // 重力加速度
  Eigen::Vector3d externalForce = getExternalForce();  // 外部力
  
  // 假设空气阻力与速度成正比，阻力系数 b 需要根据飞行器的特性来设置
  // double drag_coefficient = 0.1;  // 假定阻力系数为 0.1，实际值需要根据飞行器的空气动力学特性确定
  // Eigen::Vector3d dragForce = -drag_coefficient * x_dot;  // 空气阻力
  // 计算重力，假设重力方向是负 z 轴
  Eigen::Vector3d gravityForce = -mass * gravity * Eigen::Vector3d(0, 0, 1);  // 重力方向

  // 计算加速度：根据牛顿第二定律 F = m * a => a = F / m
  Eigen::Vector3d totalForce = gravityForce + resistance * vnorm + externalForce + R * thrust * Eigen::Vector3d(0, 0, 1);

  // 计算线速度的导数（加速度）
  // v_dot = totalForce / mass;
  v_dot =(getExternalForce()+R *thrust * Eigen::Vector3d(0, 0, 1) - Eigen::Vector3d(0, 0, mass_ * g_) - resistance * vnorm) / mass_;
  acc_ = v_dot;

  R_dot = R * omega_vee;
  //请在这里补充完四旋翼飞机的动力学模型，角速度导数的计算涉及到惯性矩阵J_的逆、力矩、科里奥利力（通过角速度与惯性矩阵和角速度的叉积来计算）和外部力矩等因素。
  
  // 获取飞行器的惯性矩阵
  Eigen::Matrix3d J = getInertia();  // 惯性矩阵
  Eigen::Matrix3d J_inv = J.inverse();  // 惯性矩阵的逆  vnorm = cur_state.v;

  // 获取外部力矩
  Eigen::Vector3d externalMoment = getExternalMoment();  // 外部力矩
  // 获取力矩系数
  // double kms = getPropellerMomentCoefficient();
  // 获取平均转速
  // double rpm = (getMaxRPM() + getMinRPM())/2
  // 转置
  // Eigen::Vector3d omega;
  // omega << omega_vee(2, 1), omega_vee(0, 2), omega_vee(1, 0);
  // 计算科里奥利力项： omega × (J * omega)
  Eigen::Vector3d coriolisTerm = omega_vee * J * cur_state.omega;
  // 总力矩：包括外部力矩
  Eigen::Vector3d M_total = externalMoment + moments; 

  // 计算角速度导数： omega_dot = J_inv * (M_total - omega × (J * omega))
  // omega_dot = J_inv * (M_total - coriolisTerm); // 1x3矩阵
   
  omega_dot = J_.inverse() * (moments + getExternalMoment() - omega_vee * J_ * cur_state.omega);
  
  motor_rpm_dot = (input_ - cur_state.motor_rpm) / motor_time_constant_;

  for (int i = 0; i < 3; i++)
  {
    dxdt[0 + i]  = x_dot(i);
    dxdt[3 + i]  = v_dot(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }
  for (int i = 0; i < 4; i++)
  {
    dxdt[18 + i] = motor_rpm_dot(i);
  }
  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
      //      std::cout << "nan apply to 0 for " << i << std::endl;
    }
  }
}

void Quadrotor::setInput(double u1, double u2, double u3, double u4)  //用于设置四旋翼飞行器的输入
{
  input_(0) = u1;
  input_(1) = u2;
  input_(2) = u3;
  input_(3) = u4;
  // std::cout<<input_;
  for (int i = 0; i < 4; i++)
  {
    if (std::isnan(input_(i)))
    {
      input_(i) = (max_rpm_ + min_rpm_) / 2;
      std::cout << "NAN input ";
    }
    if (input_(i) > max_rpm_)
      input_(i) = max_rpm_;
    else if (input_(i) < min_rpm_)
      input_(i) = min_rpm_;
  }
}

const Quadrotor::State& Quadrotor::getState(void) const
{
  return state_;
}

void Quadrotor::setState(const Quadrotor::State& state)    //用于获取四旋翼飞行器当前的外部可观察状态
{
  state_.x         = state.x;
  state_.v         = state.v;
  state_.R         = state.R;
  state_.omega     = state.omega;
  state_.motor_rpm = state.motor_rpm;

  updateInternalState();
}

void Quadrotor::setStatePos(const Eigen::Vector3d& Pos)   //设置四旋翼飞行器的位置状态
{
  state_.x = Pos;

  updateInternalState();
}

double Quadrotor::getMass(void) const   //获取四旋翼飞行器的质量
{
  return mass_;
}

void Quadrotor::setMass(double mass)   //设置四旋翼飞行器的质量
{
  mass_ = mass;
}

double Quadrotor::getGravity(void) const   //获取四旋翼飞行器所处环境的重力加速度
{
  return g_;
}


void Quadrotor::setGravity(double g)   //用于设置四旋翼飞行器所处环境的重力加速度
{
  g_ = g;
}

const Eigen::Matrix3d& Quadrotor::getInertia(void) const   //用于获取四旋翼飞行器的惯性矩阵
{
  return J_;
}

void Quadrotor::setInertia(const Eigen::Matrix3d& inertia)   //设置四旋翼飞行器的惯性矩阵
{
  if (inertia != inertia.transpose())
  {
    std::cerr << "Inertia matrix not symmetric, not setting" << std::endl;
    return;
  }
  J_ = inertia;
}

double Quadrotor::getArmLength(void) const   //用于获取四旋翼飞行器的螺旋桨臂长
{
  return arm_length_;
}


void Quadrotor::setArmLength(double d)    //用于设置四旋翼飞行器的螺旋桨臂长
{
  if (d <= 0)
  {
    std::cerr << "Arm length <= 0, not setting" << std::endl;
    return;
  }

  arm_length_ = d;
}

double Quadrotor::getPropRadius(void) const   //设置四旋翼飞行器的螺旋桨半径
{
  return prop_radius_;
}

void Quadrotor::setPropRadius(double r)    //用于获取四旋翼飞行器的螺旋桨半径
{
  if (r <= 0)
  {
    std::cerr << "Prop radius <= 0, not setting" << std::endl;
    return;
  }
  prop_radius_ = r;
}

double Quadrotor::getPropellerThrustCoefficient(void) const   //用于获取四旋翼飞行器的螺旋桨推力系数
{
  return kf_;
}


void Quadrotor::setPropellerThrustCoefficient(double kf)   //用于设置四旋翼飞行器的螺旋桨推力系数
{
  if (kf <= 0)
  {
    std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
    return;
  }

  kf_ = kf;
}

double Quadrotor::getPropellerMomentCoefficient(void) const    //获取四旋翼飞行器的螺旋桨力矩系数
{
  return km_;
}


void Quadrotor::setPropellerMomentCoefficient(double km)   //设置四旋翼飞行器的螺旋桨力矩系数
{
  if (km <= 0)
  {
    std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
    return;
  }

  km_ = km;
}

double Quadrotor::getMotorTimeConstant(void) const    //获取四旋翼飞行器的电机时间常数
{
  return motor_time_constant_;
}

void Quadrotor::setMotorTimeConstant(double k)    //用于设置四旋翼飞行器的电机时间常数
{
  if (k <= 0)
  {
    std::cerr << "Motor time constant <= 0, not setting" << std::endl;
    return;
  }

  motor_time_constant_ = k;
}

const Eigen::Vector3d& Quadrotor::getExternalForce(void) const    //用于获取四旋翼飞行器所受的外部力
{
  return external_force_;
}


void Quadrotor::setExternalForce(const Eigen::Vector3d& force)   //用于设置四旋翼飞行器所受的外部力
{
  external_force_ = force;
}

const Eigen::Vector3d& Quadrotor::getExternalMoment(void) const    //用于获取四旋翼飞行器所受的外部力矩
{
  return external_moment_;
}

void Quadrotor::setExternalMoment(const Eigen::Vector3d& moment)   //用于设置四旋翼飞行器所受的外部力矩
{
  external_moment_ = moment;
}

double Quadrotor::getMaxRPM(void) const     //用于获取四旋翼飞行器电机的最大转速
{
  return max_rpm_;
}

void Quadrotor::setMaxRPM(double max_rpm)   
{
  if (max_rpm <= 0)
  {
    std::cerr << "Max rpm <= 0, not setting" << std::endl;
    return;
  }
  max_rpm_ = max_rpm;
}

double Quadrotor::getMinRPM(void) const  //用于获取四旋翼飞行器电机的最小转速
{
  return min_rpm_;
}

void Quadrotor::setMinRPM(double min_rpm)
{
  if (min_rpm < 0)
  {
    std::cerr << "Min rpm < 0, not setting" << std::endl;
    return;
  }
  min_rpm_ = min_rpm;
}



Eigen::Vector3d Quadrotor::getAcc() const
{
  return acc_;
}

}