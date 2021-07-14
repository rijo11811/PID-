#ifndef PID_H
#define PID_H
#include <vector>
#include <numeric>
class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();


  void UpdateCoefficients();
 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double prev_control;


  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double delta_plant = 0;
  double delta_u = 0;
  double Kd;
  std::vector<double> cte_vec;

};

#endif  // PID_H
