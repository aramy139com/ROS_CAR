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
      float linear_x;				//X�᷽�� �ٶ� m/s
      float angular_z;			//Z�᷽�����ת�ٶ�   ����/��
    };
    //void initialize(int motor_max_rpm, float perimeter, float base_width, int pwm_bits);
		void initialize(float perimeter, float base_width);
    output getRPM(float linear_x, float angular_z);

  private:
    float linear_vel_x_mins_;					//���ٶ�  ��/����
    float angular_vel_z_mins_;				//���ٶ�  ����/����
    float tangential_vel_;						//��ת ��Ӧ������Ӧ�õ����ٶ�   ÿ���Ӷ�����
    float x_rpm_;								//���ٶȶ�Ӧ��ת��  ÿ���Ӷ���ת
    float tan_rpm_;							//��ת ��Ӧ������Ӧ�õ�ת��  ÿ���Ӷ���ת
    //int max_rpm_;								//���ת��   rpm  ÿ���Ӷ���ת
    double wheel_perimeter;			//���ӵ��ܳ���תһȦ�ľ��� ��λ ��
    float base_width_;					//������  ��λ ��
    //double pwm_res_;
};

#endif
