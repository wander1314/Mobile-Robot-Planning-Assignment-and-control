#include <iostream>
#include <so3_control/SO3Control.h>

#include <ros/ros.h>

SO3Control::SO3Control()
  : mass_(0.5)
  , g_(9.81)
{
  acc_.setZero();
}

void
SO3Control::setMass(const double mass)
{
  mass_ = mass;
}

void
SO3Control::setGravity(const double g)
{
  g_ = g;
}

void
SO3Control::setPosition(const Eigen::Vector3d& position)
{
  pos_ = position;
}

void
SO3Control::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

void
SO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)   //用于根据期望的位置、速度、加速度、偏航角及其变化率以及控制增益等信息，计算出控制所需的力向量 force_ 和姿态四元数 orientation_
{

  Eigen::Vector3d totalError =
    (des_pos - pos_) + (des_vel - vel_) + (des_acc - acc_);

  ros::Time currentTime = ros::Time::now();
  // recorderr(currentTime, des_pos, pos_);
  if(!dataFile.is_open()){
    dataFile.open("/home/stuwork/Mobile-Robot-Planning-Assignment-and-control/src/quadrotor_simulator/so3_control/src/control_data.txt", std::ios::out|std::ios::trunc);
  }

  dataFile << currentTime << " ";
  dataFile << des_pos(0) << " " << des_pos(1) << " " << des_pos(2) << " ";
  dataFile << pos_(0) << " " << pos_(1) << " " << pos_(2) << "\n";

  if((pos_-Eigen::Vector3d(10.0, -4.0, 0.0)).norm() <= 2.0){
  if(!dataFile_time.is_open())
  {
    dataFile_time.open("/home/stuwork/Mobile-Robot-Planning-Assignment-and-control/src/quadrotor_simulator/so3_control/src/control_timedata.txt", std::ios::out|std::ios::trunc);
    dataFile_time << currentTime << " ";
  }

  

  }
  

  Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                     fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                     fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

  force_.noalias() =
    kx.asDiagonal() * (des_pos - pos_) + kv.asDiagonal() * (des_vel - vel_) +
    mass_ * /*(Eigen::Vector3d(1, 1, 1) - ka).asDiagonal() **/ (des_acc) +
    mass_ * ka.asDiagonal() * (des_acc - acc_) +
    mass_ * g_ * Eigen::Vector3d(0, 0, 1);

  // Limit control angle to 45 degree
  double          theta = M_PI / 2;
  double          c     = cos(theta);
  Eigen::Vector3d f;
  f.noalias() = kx.asDiagonal() * (des_pos - pos_) +
                kv.asDiagonal() * (des_vel - vel_) + //
                mass_ * des_acc +                    //
                mass_ * ka.asDiagonal() * (des_acc - acc_);
  if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
  {
    double nf        = f.norm();
    double A         = c * c * nf * nf - f(2) * f(2);
    double B         = 2 * (c * c - 1) * f(2) * mass_ * g_;
    double C         = (c * c - 1) * mass_ * mass_ * g_ * g_;
    double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  }
  // Limit control angle to 45 degree

  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

  if (force_.norm() > 1e-6)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;
  R << b1c, b2c, b3c;

  orientation_ = Eigen::Quaterniond(R);
}

const Eigen::Vector3d&
SO3Control::getComputedForce(void)
{
  return force_;
}

const Eigen::Quaterniond&
SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

void
SO3Control::setAcc(const Eigen::Vector3d& acc)
{
  acc_ = acc;
}
