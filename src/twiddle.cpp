#include <cmath>
#include "twiddle.h"
#include "PID.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::init(std::vector<double> p_, double factor_dp) {
    p = p_;
    dp = { (p[0]* factor_dp), (p[1] * factor_dp), (p[2] * factor_dp) };
    period_ = 3000;
    count_ = 1;    
    param_num_ = 0;
    total_cte_ = 0.0;
    best_cte_ = 999999999.9;
    state = 0;
}

void Twiddle::SaveCTE(double cte) {
    total_cte_ += pow(cte, 4);
    count_++;
    
}

std::vector<double> Twiddle::GetParams() {
    return p;    
}

bool Twiddle::updateParams() {
    double sum_dp = dp[0] + dp[1] + dp[2];  
    
    if (sum_dp < 0.001) {
        init(p,0.1);
        std::cout << "twiddle again" << std::endl;
        return false;
    }
   
    if (count_ == period_) {
        double ave_cte = total_cte_ / count_;
        count_ = 1;
        total_cte_ = 0;
        std::cout << " ave_cte: " << ave_cte << std::endl;
        switch (state)
        {
            //case(1) and case(0) for the Kp optimziation
            //case(2) and case(3) for the Ki optimization
            //case(4) and case(5) for the Kd optimization
            case(0): {
                std::cout << "here is case: " << state << std::endl;
                if (ave_cte < best_cte_) {  
                    best_cte_ = ave_cte;

                    //for next state
                    state = 2;
                    p[1] += dp[1]; 

                    //update params
                    return true;
                }
                else {
                    //try another side
                    p[0] -= 2 * dp[0];
                    state = 1;

                    return true;
                }
                break;
            }
            case(1): {

                std::cout << "here is case: " << state << std::endl;                
                if (ave_cte < best_cte_) {
                    //the first time, here would update the best_cte_ to initial cte made by initialized params.
                    //std::cout << "###update the parameters!!###" << std::endl;  
                    best_cte_ = ave_cte;

                    //for next state
                    state = 2;
                    p[1] += dp[1];                    
                    return true;
                }
                else {
                    p[0] += dp[0];
                    dp[0] *= 0.7;

                    //for next state
                    p[1] += dp[1];                    
                    state = 2;
                    return true;
                }
                break;
            }
            case(2): {
                std::cout << "here is case: " << state << std::endl;
                if (ave_cte < best_cte_) {
                    //the first time, here would update the best_cte_ to initial cte made by initialized params.
                    //std::cout << "###update the parameters!!###" << std::endl; 
                    best_cte_ = ave_cte;


                    //for next state
                    state = 4;
                    p[2] += dp[2];
                    
                    return true;
                }
                else {
                    p[1] -= 2 * dp[1];
                    state = 3;
                    return true;
                }
                break;

            }
            case(3): {
                std::cout << "here is case: " << state << std::endl;
                if (ave_cte < best_cte_) {
                    //the first time, here would update the best_cte_ to initial cte made by initialized params.
                    //std::cout << "###update the parameters!!###" << std::endl;
                    best_cte_ = ave_cte;


                    //for next state
                    state = 4;
                    p[2] += dp[2];
                   
                    return true;
                }
                else {
                    p[1] += dp[1];
                    dp[1] *= 0.6;

                    //for next state
                    p[2] += dp[2];                   
                    state = 4;
                    return true;
                }
                break;
            }
            case(4): {
                std::cout << "here is case: " << state << std::endl;
                if (ave_cte < best_cte_) {
                    //the first time, here would update the best_cte_ to initial cte made by initialized params.
                    //std::cout << "###update the parameters!!###" << std::endl;                    
                    best_cte_ = ave_cte;

                    //for next state
                    state = 0;
                    p[0] += dp[0];
                    
                    return true;
                }
                else {
                    p[2] -= 2 * dp[2];
                    state = 5;
                    return true;
                }
                break;
            }
            case(5): {
                std::cout << "here is case: " << state << std::endl;
                if (ave_cte < best_cte_) {
                    //the first time, here would update the best_cte_ to initial cte made by initialized params.
                    //std::cout << "###update the parameters!!###" << std::endl;                    
                    best_cte_ = ave_cte;

                    //for next state
                    state = 0;
                    p[0] += dp[0];
                    
                    return true;
                }
                else {
                    p[2] += dp[2];
                    dp[2] *= 0.6;

                    //for next state
                    p[0] += dp[0];                    
                    state = 0;
                    return true;
                }
                break;
            }

            default:
                break;
        }     
       
    }
    
    // if nothing happened
    return false;
    
   
}