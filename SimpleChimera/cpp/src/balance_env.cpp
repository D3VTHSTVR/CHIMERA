#include "balance_env.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace simplechimera {

BalanceEnv::BalanceEnv(BulletSimulation& sim, double maxTorque, int maxSteps)
    : sim_(sim)
    , maxTorque_(maxTorque)
    , maxSteps_(maxSteps)
    , stepCount_(0)
    , stableCount_(0)
{}

void BalanceEnv::reset() {
    sim_.reset();
    stepCount_ = 0;
    stableCount_ = 0;
}

void BalanceEnv::getState(double* state) const {
    sim_.getState(state);
}

void BalanceEnv::step(const double* action, double* next_state, double* reward, bool* done) {
    const int warmup = std::min(2000, std::max(200, maxSteps_ / 3));
    const int torque_delay = std::min(400, std::max(50, maxSteps_ / 6));
    const int ramp_steps = std::min(900, std::max(100, maxSteps_ / 2));
    const int stable_required = std::min(400, maxSteps_ / 10);

    double torques[18];
    if (stepCount_ < warmup) {
        for (int i = 0; i < 18; ++i) torques[i] = 0;
        stableCount_ = 0;
    } else {
        int after_warmup = stepCount_ - warmup;
        if (after_warmup < torque_delay) {
            for (int i = 0; i < 18; ++i) torques[i] = 0;
            stableCount_ = 0;
        } else {
            double state[40];
            sim_.getState(state);
            double base_xyz[3], base_v[3];
            sim_.getBasePosition(base_xyz);
            sim_.getBaseLinearVelocity(base_v);
            double base_z = base_xyz[2];
            double speed = std::sqrt(base_v[0]*base_v[0] + base_v[1]*base_v[1] + base_v[2]*base_v[2]);
            bool feet_contact = sim_.limbTipsInContact();
            bool upright = (state[0]*state[0] + state[1]*state[1]) < 0.5;
            bool moving_up = base_v[2] > 0;
            bool moving_fast = speed > 0.04;
            bool too_high = base_z > 0.038;
            bool would_allow = feet_contact && upright && !moving_up && !moving_fast && !too_high;
            if (would_allow) stableCount_++; else stableCount_ = 0;
            bool allow_torque = (stableCount_ >= stable_required);
            double scale = 0.04;
            if (allow_torque) {
                int into_ramp = stableCount_ - stable_required;
                scale = 0.04 + 0.96 * std::min(1.0, static_cast<double>(into_ramp) / ramp_steps);
            }
            for (int i = 0; i < 18; ++i) {
                double a = std::clamp(action[i], -1.0, 1.0);
                torques[i] = allow_torque ? (a * maxTorque_ * scale) : 0;
            }
        }
    }

    sim_.setJointTorques(torques);
    sim_.step(sim_.getTimeStep());
    stepCount_++;

    sim_.getState(next_state);
    double roll = next_state[0], pitch = next_state[1];
    double roll_dot = next_state[2], pitch_dot = next_state[3];
    double tilt = std::sqrt(roll*roll + pitch*pitch);
    double ang_speed = std::sqrt(roll_dot*roll_dot + pitch_dot*pitch_dot);

    *reward = -0.6 * tilt - 0.06 * ang_speed;
    *done = (tilt > 1.0 || stepCount_ >= maxSteps_);
}

}  // namespace simplechimera
