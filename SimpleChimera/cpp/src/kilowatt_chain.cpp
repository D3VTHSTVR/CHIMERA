#include "kilowatt_chain.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace simplechimera {

KilowattChain::KilowattChain(
    double amplitude,
    double dt_snn,
    double time_step,
    double k_couple,
    double tau,
    double I_bias,
    bool use_phase_offset
)
    : amplitude_(amplitude)
    , dt_snn_(dt_snn)
    , time_step_(time_step)
    , k_couple_(k_couple)
    , use_phase_offset_(use_phase_offset)
    , steps_per_env_step_(static_cast<int>(std::max(1.0, time_step / dt_snn)))
    , snns_{
          TwoMotorSNN(1.0, tau, dt_snn, -1.2, -1.2, 0.15, 0.15, I_bias, 0.0),
          TwoMotorSNN(1.0, tau, dt_snn, -1.2, -1.2, 0.15, 0.15, I_bias, 0.0),
          TwoMotorSNN(1.0, tau, dt_snn, -1.2, -1.2, 0.15, 0.15, I_bias, 0.0),
      }
{
    for (int i = 0; i < ACTION_DIM; ++i) {
        int joint_in_leg = i % 3;
        if (joint_in_leg == 0) joint_phase_offset_[i] = 0.0;
        else if (joint_in_leg == 1) joint_phase_offset_[i] = M_PI / 2.0;
        else joint_phase_offset_[i] = M_PI;
    }
    snns_[1].set_initial_voltage(0.6, 0.3);
    snns_[2].set_initial_voltage(0.3, 0.6);
}

void KilowattChain::reset() {
    for (auto& s : snns_) s.reset();
    snns_[1].set_initial_voltage(0.6, 0.3);
    snns_[2].set_initial_voltage(0.3, 0.6);
}

void KilowattChain::step(const double* state, double* action) {
    (void)state;
    double m1_f, m2_f, m1_m, m2_m, m1_r, m2_r;

    for (int i = 0; i < steps_per_env_step_; ++i)
        snns_[0].step(&m1_f, &m2_f);
    snns_[0].get_motor_commands(&m1_f, &m2_f);

    const double drive_mid = k_couple_ * (m1_f + m2_f);
    const double orig_mid = snns_[1].I_bias;
    snns_[1].I_bias = orig_mid + drive_mid;
    for (int i = 0; i < steps_per_env_step_; ++i)
        snns_[1].step(&m1_m, &m2_m);
    snns_[1].I_bias = orig_mid;
    snns_[1].get_motor_commands(&m1_m, &m2_m);

    const double drive_rear = k_couple_ * (m1_m + m2_m);
    const double orig_rear = snns_[2].I_bias;
    snns_[2].I_bias = orig_rear + drive_rear;
    for (int i = 0; i < steps_per_env_step_; ++i)
        snns_[2].step(&m1_r, &m2_r);
    snns_[2].I_bias = orig_rear;
    snns_[2].get_motor_commands(&m1_r, &m2_r);

    const double leg_motors[6] = { m1_f, m1_m, m1_r, m2_f, m2_m, m2_r };
    for (int i = 0; i < ACTION_DIM; ++i) {
        const int leg = i / 3;
        const double base_phase = M_PI * (2.0 * leg_motors[leg] - 1.0);
        double out;
        if (use_phase_offset_) {
            const double phase = base_phase + joint_phase_offset_[i];
            out = amplitude_ * std::sin(phase);
        } else {
            out = amplitude_ * (2.0 * leg_motors[leg] - 1.0);
        }
        action[i] = std::clamp(out, -1.0, 1.0);
    }
}

}  // namespace simplechimera
