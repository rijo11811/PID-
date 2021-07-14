#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   d_error = 0;
   i_error = 0;
   p_error = 0;
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   delta_plant = 0;
   delta_u = 0;



}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   if(prev_cte == 0)
   {
   d_error = 0;
   }
   else
   {
   d_error = cte - prev_cte;
   }
   if(cte_vec.size()==10){cte_vec.erase(cte_vec.begin());cte_vec.push_back(cte);}
   else{cte_vec.push_back(cte);}
   i_error = std::accumulate(cte_vec.begin(),cte_vec.end(),0.0);
   p_error = cte;
   delta_plant = prev_cte - cte;
   prev_cte = cte;
   //std::cout<<"PID error "<<p_error<<" "<<i_error<<" "<<d_error<<"\n";

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    double control =0.0;
    control = -Kp*p_error-Kd*d_error-Ki*i_error;
    delta_u = prev_control - control;
    if(delta_u == 0)
    {
    std::cout<<"*********************************************************************************";
    delta_u = 0.0001;
    }
    //std::cout<<"delta_u:"<<delta_u<<" control:"<<control<<"\n";
    prev_control = control;
    return control;

  return 0.0;  // TODO: Add your total error calc here!
}

void PID::UpdateCoefficients()
{
double v = delta_plant/delta_u;
double alpha = 0.0001;
Kp=Kp - alpha*p_error*p_error*v;
Ki=Ki - alpha*p_error*i_error*v;
Kd=Kd - alpha*p_error*d_error*v;
std::cout<<"Kp:"<<Kp<<" Ki:"<<Ki<<" Kd:"<<Kd<<"\n";
}





