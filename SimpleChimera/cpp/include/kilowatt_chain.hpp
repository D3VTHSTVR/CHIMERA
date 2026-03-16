#ifndef SIMPLECHIMERA_KILOWATT_CHAIN_HPP
#define SIMPLECHIMERA_KILOWATT_CHAIN_HPP

#include "two_motor_snn.hpp"
#include <array>

namespace simplechimera {

/** State dimension from balance env (roll, pitch, roll_dot, pitch_dot + 18*2). */
constexpr int STATE_DIM = 40;
/** Action dimension: 18 joint torques. */
constexpr int ACTION_DIM = 18;

/**
 * Chain of 3 TwoMotorSNN modules (front, middle, rear).
 * Outputs 18 joint torques in [-1, 1] for SimpleChimera BalanceEnv.
 * Joint order: L1_coxa,femur,tibia, L2_..., L3_..., R1_..., R2_..., R3_....
 */
class KilowattChain {
public:
    KilowattChain(
        double amplitude = 0.5,
        double dt_snn = 0.01,
        double time_step = 0.002,
        double k_couple = 0.1,
        double tau = 0.05,
        double I_bias = 0.95,
        bool use_phase_offset = true
    );

    void reset();
    /**
     * One env step: step SNNs with coupling, fill action[18] in [-1, 1].
     * state[40] is ignored (API compatibility with state-based policies).
     */
    void step(const double* state, double* action);

private:
    double amplitude_;
    double dt_snn_;
    double time_step_;
    double k_couple_;
    bool use_phase_offset_;
    int steps_per_env_step_;
    std::array<double, ACTION_DIM> joint_phase_offset_;
    std::array<TwoMotorSNN, 3> snns_;
};

}  // namespace simplechimera

#endif
