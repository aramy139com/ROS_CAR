#include "Kinematics.h"

void Kinematics::initialize(float perimeter, float base_width)
{
  wheel_perimeter = perimeter;
  //max_rpm_ = motor_max_rpm;
  base_width_ = base_width;
  //pwm_res_ = pow((float)2, pwm_bits) - 1;
}
//入口为 要求的线速度 米/秒  和期望的 角速度 弧度/秒 
//出口为 期望的 左轮 和 右轮 的转速		单位 转/分钟
Kinematics::output Kinematics::getRPM(float linear_x, float angular_z){
  //convert m/s to m/min
  linear_vel_x_mins_ = linear_x * 60;
  //convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;

  //Vt = ω * radius
  tangential_vel_ = angular_vel_z_mins_ * base_width_;

  x_rpm_ = linear_vel_x_mins_ / wheel_perimeter;
  tan_rpm_ = tangential_vel_ / wheel_perimeter;

  Kinematics::output rpm;

  //calculate for the target motor RPM and direction
  //front-left motor
  rpm.leftmotor = x_rpm_ - tan_rpm_;
  //rear-left motor
  rpm.rightmotor = x_rpm_ + tan_rpm_;
  return rpm;
}