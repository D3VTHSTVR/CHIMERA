#ifndef SIMPLECHIMERA_GUI_VIEWER_HPP
#define SIMPLECHIMERA_GUI_VIEWER_HPP

namespace simplechimera {

class BulletSimulation;
class BalanceEnv;
class KilowattChain;

/** Run simulation with a visible window. Returns total reward. Requires GLFW/OpenGL. */
double runSimulationWithGui(
    BulletSimulation& sim,
    BalanceEnv& env,
    KilowattChain& controller,
    int maxSteps,
    double amplitude
);

}  // namespace simplechimera

#endif
