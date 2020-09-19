#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>
#include <cmath>

class PID {
 public:
 

/**
* PID Coefficients
	*/
  double Kp;
  double Ki;
  double Kd;

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
  void Init(std::vector<double> p_);

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
  double TotalError_speed();

  void UpdateParms(std::vector<double> p_);

 private:
  /**
   * PID Errors
   */
  bool initialization = false;
  double p_error;
  double i_error;  
  double d_error;
  int count;
  double prev_cte_2;
  double prev_cte_3;
  double temp_2;
  double inte_cte;
  double prev_cte;
  int step;
};

#endif  // PID_H