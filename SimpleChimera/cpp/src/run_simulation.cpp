/**
 * Full C++ run: Bullet simulation + Kilowatt controller. Replaces Python run.py.
 *
 * Usage: ./run_simulation [--gui] [--steps N] [--amplitude A]
 *   --gui  show a window with the simulation (requires GLFW)
 */

#include "balance_env.hpp"
#include "bullet_simulation.hpp"
#include "kilowatt_chain.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#ifdef HAVE_GLFW
#include "gui_viewer.hpp"
#endif

int main(int argc, char** argv) {
    int maxSteps = 5000;
    double amplitude = 0.5;
    bool gui = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--gui") == 0) {
            gui = true;
        } else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
            maxSteps = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--amplitude") == 0 && i + 1 < argc) {
            amplitude = std::atof(argv[++i]);
        }
    }

    simplechimera::BulletSimulation sim(0.002, 1.0, 3.2, 400, 250, 100);
    simplechimera::BalanceEnv env(sim, 0.0002, maxSteps);
    simplechimera::KilowattChain controller(amplitude, 0.01, 0.002, 0.1, 0.05, 0.95, true);

    env.reset();
    controller.reset();

    double totalReward = 0;

#ifdef HAVE_GLFW
    if (gui) {
        totalReward = simplechimera::runSimulationWithGui(sim, env, controller, maxSteps, amplitude);
        std::printf("Steps: (see window)  Total reward: %.2f\n", totalReward);
        return 0;
    }
#endif
    if (gui) {
        std::fprintf(stderr, "GUI not built: install GLFW (e.g. brew install glfw) and rebuild.\n");
        return 1;
    }

    double state[simplechimera::STATE_DIM];
    double action[simplechimera::ACTION_DIM];
    int step = 0;
    sim.getState(state);
    while (step < maxSteps) {
        controller.step(state, action);
        double reward;
        bool done;
        env.step(action, state, &reward, &done);
        totalReward += reward;
        step++;
        if (done) break;
    }

    std::printf("Steps: %d  Total reward: %.2f\n", step, totalReward);
    return 0;
}
