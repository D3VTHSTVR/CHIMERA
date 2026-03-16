#ifndef SIMPLECHIMERA_TWO_MOTOR_SNN_HPP
#define SIMPLECHIMERA_TWO_MOTOR_SNN_HPP

namespace simplechimera {

/**
 * Two-neuron SNN for alternating two motors (walk pattern).
 * N1 and N2 with reciprocal inhibition; N1 -> motor1, N2 -> motor2.
 * Outputs (motor1, motor2) in [0, 1] for duty cycle / PWM.
 */
class TwoMotorSNN {
public:
    TwoMotorSNN(
        double V_th = 1.0,
        double tau = 0.05,
        double dt = 0.01,
        double W_12 = -1.2,
        double W_21 = -1.2,
        double W_11 = 0.15,
        double W_22 = 0.15,
        double I_bias = 0.6,
        double V_reset = 0.0
    );

    void reset();
    /** Set initial membrane potentials (e.g. for phase offset in chain). */
    void set_initial_voltage(double V1, double V2);
    /** One time step; returns (motor1, motor2) in [0, 1]. */
    void step(double* motor1, double* motor2);
    /** Current motor outputs; call after step(). */
    void get_motor_commands(double* motor1, double* motor2) const;

    double I_bias;  // exposed for coupling (front->mid->rear)

private:
    double V_th_, tau_, dt_;
    double W_12_, W_21_, W_11_, W_22_, V_reset_;
    int refrac_steps_;
    double V1_, V2_, s1_, s2_, alpha_;
    int refrac1_, refrac2_;
};

}  // namespace simplechimera

#endif
