#ifndef SIMPLECHIMERA_BALANCE_ENV_HPP
#define SIMPLECHIMERA_BALANCE_ENV_HPP

#include "bullet_simulation.hpp"

namespace simplechimera {

/**
 * Balance environment: same logic as Python BalanceEnv.
 * Warmup, torque delay, ramp; reward = upright + tripod; done on tilt or timeout.
 */
class BalanceEnv {
public:
    BalanceEnv(
        BulletSimulation& sim,
        double maxTorque = 0.0002,
        int maxSteps = 2500
    );

    void reset();
    /** action[18] in [-1, 1]; returns next_state, reward, done. */
    void step(const double* action, double* next_state, double* reward, bool* done);

    int getStateDim() const { return BulletSimulation::STATE_DIM; }
    int getActionDim() const { return BulletSimulation::N_JOINTS; }
    int getStepCount() const { return stepCount_; }

private:
    void getState(double* state) const;

    BulletSimulation& sim_;
    double maxTorque_;
    int maxSteps_;
    int stepCount_;
    int stableCount_;
};

}  // namespace simplechimera

#endif
