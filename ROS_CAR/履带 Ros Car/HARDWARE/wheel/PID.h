#ifndef PID_H
#define PID_H

#include "config.h"

#define constrain(amt,low,high) \
	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class PID
{
  public:
		
    void initialize(float min_val, float max_val, float kp, float ki, float kd);
    double computeLoc(float setpoint, float measured_value);			//位置PID
		//double computeInc(float setpoint, float measured_value);			//增量PID
    void updateConstants(float kp, float ki, float kd);
		void tostring(char *buf);

  private:
    float min_val_;
    float max_val_;
    float kp_;
    float ki_;
    float kd_;
    double integral_;			//累计错误值
    double derivative_;		
    double prev_error_;		//上一次偏差值
		double prev_error2;		//上上次偏差值
};

#endif
