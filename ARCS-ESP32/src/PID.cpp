#include "PID.h"

/*
 * : Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    for (int i = 0; i < rollingAccumulatorSize; i++) {
        rollingaccumulator[i] = 0.0;
    }
    rollingindex = rollingAccumulatorSize - 1;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;

    if (Ki != 0.0) {
        rollingindex = (rollingindex + 1) % rollingAccumulatorSize;
        double head  = rollingaccumulator[rollingindex];
        rollingaccumulator[rollingindex] = cte ;
        i_error += cte - head;
    } else {
        i_error += cte;
    }
}


double PID::TotalError() {
    return Kp * p_error + Ki * i_error + Kd * d_error;
}
