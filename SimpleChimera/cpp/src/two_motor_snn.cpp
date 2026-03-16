#include "two_motor_snn.hpp"
#include <algorithm>
#include <cmath>

namespace simplechimera {

TwoMotorSNN::TwoMotorSNN(
    double V_th, double tau, double dt,
    double W_12, double W_21, double W_11, double W_22,
    double I_bias, double V_reset
)
    : I_bias(I_bias)
    , V_th_(V_th)
    , tau_(tau)
    , dt_(dt)
    , W_12_(W_12)
    , W_21_(W_21)
    , W_11_(W_11)
    , W_22_(W_22)
    , V_reset_(V_reset)
    , refrac_steps_(static_cast<int>(std::max(1.0, 0.08 / dt)))
    , V1_(0.92)
    , V2_(0.0)
    , s1_(0.0)
    , s2_(0.0)
    , alpha_(0.9)
    , refrac1_(0)
    , refrac2_(0)
{}

void TwoMotorSNN::reset() {
    V1_ = 0.92;
    V2_ = 0.0;
    s1_ = s2_ = 0.0;
    refrac1_ = refrac2_ = 0;
}

void TwoMotorSNN::set_initial_voltage(double V1, double V2) {
    V1_ = V1;
    V2_ = V2;
}

void TwoMotorSNN::step(double* motor1, double* motor2) {
    const double tau = std::max(tau_, dt_ * 2.0);
    const double spike1 = (V1_ >= V_th_) ? 1.0 : 0.0;
    const double spike2 = (V2_ >= V_th_) ? 1.0 : 0.0;
    const double I1 = I_bias + W_11_ * spike1 + W_21_ * spike2;
    const double I2 = I_bias + W_22_ * spike2 + W_12_ * spike1;

    if (refrac1_ <= 0) {
        V1_ += (I1 - V1_) / tau * dt_;
    } else {
        V1_ = V_reset_;
        --refrac1_;
    }
    if (refrac2_ <= 0) {
        V2_ += (I2 - V2_) / tau * dt_;
    } else {
        V2_ = V_reset_;
        --refrac2_;
    }

    if (V1_ >= V_th_) {
        V1_ = V_reset_;
        refrac1_ = refrac_steps_;
    }
    if (V2_ >= V_th_) {
        V2_ = V_reset_;
        refrac2_ = refrac_steps_;
    }

    V1_ = std::clamp(V1_, -0.5, V_th_ + 0.2);
    V2_ = std::clamp(V2_, -0.5, V_th_ + 0.2);

    const double rate1 = std::clamp(V1_ / V_th_, 0.0, 1.0);
    const double rate2 = std::clamp(V2_ / V_th_, 0.0, 1.0);
    s1_ = alpha_ * s1_ + (1.0 - alpha_) * rate1;
    s2_ = alpha_ * s2_ + (1.0 - alpha_) * rate2;

    if (motor1) *motor1 = s1_;
    if (motor2) *motor2 = s2_;
}

void TwoMotorSNN::get_motor_commands(double* motor1, double* motor2) const {
    if (motor1) *motor1 = s1_;
    if (motor2) *motor2 = s2_;
}

}  // namespace simplechimera
