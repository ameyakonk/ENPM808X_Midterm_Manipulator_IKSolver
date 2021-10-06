#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H
#include <bits/stdc++.h>
#include <iostream>

class Control_System
{
   private:
           double kp;
           double ki;
	   double kd;
	   double sampling_time;
	   double current_velocity;
	   double target_velocity;
	   double current_theta;
           double target_theta;
   public:
           void insert_PID_Params(double,double,double,double);
           double compute_PID(double,double);

};

void Control_System::insert_PID_Params(double k_p, double k_i, double k_d, double s_t)
{
	kp = k_p;
	ki = k_i;
	kd = k_d;
	sampling_time = s_t;
}

double Control_System::compute_PID(double target_theta, double current_theta)
{
        return 0;
}

#endif
