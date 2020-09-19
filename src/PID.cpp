#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(std::vector<double> p_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	PID::Kp = p_[0]; 
	PID::Ki = p_[1];
	PID::Kd = p_[2];
	count = 0;
	prev_cte_2 = 0;
	prev_cte_3 = 0;
	temp_2 = 0;
	p_error = 0;
	i_error = 0;
	d_error = 0;	
	inte_cte = 0.0;
	prev_cte = 0.0;
	step = 0;
	initialization = true;

}

void PID::UpdateParms(std::vector<double> p_) {
	PID::Kp = p_[0];
	PID::Ki = p_[1];
	PID::Kd = p_[2];

}


void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	if (initialization) {
		if (step == 0) {
			p_error = pow(cte,3);
			i_error = 0;
			d_error = 0;			
			prev_cte = cte;
			step += 1;
		}
		else {
			
			p_error = cte;
			//p_error = cte;			
			
			d_error = cte - prev_cte;
			if (step % 10 == 0) {
				i_error += 3.0 * cte + 4.0 * prev_cte + 3.0 * prev_cte_2 + 0.0 * prev_cte_3;
			}
			
			
			double limit_of_Ki = 18;

			//control the limit of I
			if (i_error > limit_of_Ki) {
				i_error = limit_of_Ki;				
			}
			else if(i_error < -limit_of_Ki){
				i_error = -limit_of_Ki;				
			}
			
			if (step < 500) {
				i_error = 0;
			}
			prev_cte_3 = prev_cte_2;
			prev_cte_2 = prev_cte;
			prev_cte = cte;
			step += 1;

		}
		
	}
}
double PID::TotalError() {
	//std::cout << "i_error: " << i_error << std::endl;
	return -Kp * p_error - Ki * i_error - Kd * d_error;  // TODO: Add your total error calc here!
}
double PID::TotalError_speed() {
	//std::cout << "i_error_speed: " << i_error << std::endl;
	return -Kp * fabs(p_error) - Ki * fabs(i_error) - Kd * fabs(d_error);  // TODO: Add your total error calc here!
}