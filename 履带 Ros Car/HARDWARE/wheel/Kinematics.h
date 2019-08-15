#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "config.h"

class Kinematics
{
  public:
    struct output {
      int leftmotor;
      int rightmotor;
    };
    struct velocities {
      float linear_x;				//X轴方向 速度 m/s
      float angular_z;			//Z轴方向的旋转速度   弧度/秒
    };
    //void initialize(int motor_max_rpm, float perimeter, float base_width, int pwm_bits);
		void initialize(float perimeter, float base_width);
    output getRPM(float linear_x, float angular_z);

  private:
    float linear_vel_x_mins_;					//线速度  米/分钟
    float angular_vel_z_mins_;				//角速度  弧度/分钟
    float tangential_vel_;						//旋转 对应的轮子应该的线速度   每分钟多少米
    float x_rpm_;								//线速度对应的转速  每分钟多少转
    float tan_rpm_;							//旋转 对应的轮子应该的转速  每分钟多少转
    //int max_rpm_;								//最大转速   rpm  每分钟多少转
    double wheel_perimeter;			//轮子的周长，转一圈的距离 单位 米
    float base_width_;					//车体宽度  单位 米
    //double pwm_res_;
};

#endif
