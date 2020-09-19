#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <iostream>


class Twiddle {
private:
	int count_;
	int period_;	
	int param_num_;
	double total_cte_;
	double best_cte_;
	int state;

	std::vector<double> p;
	std::vector<double> dp;

public:

	/**
	 * Constructor.
	 */
	Twiddle();

	/*
	* Destructor.
	*/
	virtual ~Twiddle();

	/**
	 * Initialize Twiddle.
	 * @param Kp Proportional - to minimize CTE
	 * @param Ki Integral - to adjust for steering drift
	 * @param Kd Differential - to avoid overshooting
	 */
	void init(std::vector<double> p_, double factor_dp);

	/**
	 * Increment twiddle count and total cross track error
	 * @param cte Cross Track Error value to sum
	 */
	void SaveCTE(double cte);

	/**
	 * get the current twiddle parameters
	 */
	std::vector<double> GetParams();


	/**
	 * Evaluates error (sum(cte) / count) and runs the twiddle algorithm to adjust initial parameter values.
	 * @return vector with updated {Kp, Ki, Kd} values
	 */
	bool updateParams();
};

#endif /* TWIDDLE_H */