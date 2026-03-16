/**
 * Run SimpleChimera with MuJoCo: same Kilowatt controller, MuJoCo physics and viewer.
 * Builds only when MuJoCo is found. Usage: ./run_mujoco [--model path] [--steps N] [--amplitude A]
 *
 * Install MuJoCo: pip install mujoco
 * Then set MUJOCO_PATH to the package dir, e.g.:
 *   export MUJOCO_PATH=$(python -c "import mujoco; print(mujoco.__path__[0])")
 *   cd build && cmake .. -DMUJOCO_PATH=$MUJOCO_PATH && cmake --build .
 */

#include "kilowatt_chain.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr int STATE_DIM = simplechimera::STATE_DIM;
constexpr int ACTION_DIM = simplechimera::ACTION_DIM;
// Torque scale: 0.00028 matches PyBullet but is too small to see in MuJoCo; use ~0.02 so the robot moves visibly.
constexpr double MAX_TORQUE = 0.02;

// MuJoCo free joint: qpos[0:3]=position(x,y,z), qpos[3:7]=quat(w,x,y,z), then qpos[7:25]=18 joint angles.
// qvel[0:3]=linear, qvel[3:6]=angular, qvel[6:24]=18 joint velocities.
void mj_state_to_balance(const mjModel* m, const mjData* d, double* state) {
    const double* q = d->qpos;
    const double* v = d->qvel;
    double qw = q[3], qx = q[4], qy = q[5], qz = q[6];
    double roll  = std::atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
    double pitch = std::asin(2*(qw*qy - qz*qx));
    if (std::abs(std::cos(pitch)) < 1e-8) pitch = 0;
    state[0] = roll;
    state[1] = pitch;
    state[2] = v[3];
    state[3] = v[4];
    for (int i = 0; i < 18; i++) {
        state[4 + i] = q[7 + i];
        state[22 + i] = v[6 + i];
    }
}

void balance_action_to_mj(const double* action, mjData* d) {
    for (int i = 0; i < 18; i++)
        d->ctrl[i] = action[i] * MAX_TORQUE;
}

std::string findModelPath(const char* argv0, const char* optPath) {
    if (optPath && optPath[0] != '\0') return optPath;
    std::string exe(argv0);
    size_t slash = exe.find_last_of("/\\");
    std::string dir = (slash != std::string::npos) ? exe.substr(0, slash) : ".";
    std::string candidates[] = {
        dir + "/../robot/six_leg_insect.xml",
        dir + "/../../robot/six_leg_insect.xml",
        "robot/six_leg_insect.xml",
        "six_leg_insect.xml",
    };
    for (const auto& p : candidates) {
        FILE* f = std::fopen(p.c_str(), "r");
        if (f) { std::fclose(f); return p; }
    }
    return "";
}

}  // namespace

int main(int argc, char** argv) {
    const char* modelPath = nullptr;
    int maxSteps = 5000;
    double amplitude = 0.5;
    bool testMotion = false;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--model") == 0 && i + 1 < argc)
            modelPath = argv[++i];
        else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc)
            maxSteps = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--amplitude") == 0 && i + 1 < argc)
            amplitude = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--test-motion") == 0)
            testMotion = true;
    }

    std::string path = findModelPath(argv[0], modelPath);
    if (path.empty()) {
        std::fprintf(stderr, "run_mujoco: could not find six_leg_insect.xml. Use --model /path/to/six_leg_insect.xml\n");
        return 1;
    }

    char err[512];
    mjModel* m = mj_loadXML(path.c_str(), nullptr, err, sizeof(err));
    if (!m) {
        std::fprintf(stderr, "run_mujoco: failed to load model: %s\n", err);
        return 1;
    }
    mjData* d = mj_makeData(m);

    // Initial pose: base at (0,0,0.102), identity quat. MuJoCo free joint: qpos[0:3]=pos, qpos[3:7]=quat(w,x,y,z).
    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] = 0.102;
    d->qpos[3] = 1;
    d->qpos[4] = 0;
    d->qpos[5] = 0;
    d->qpos[6] = 0;
    mj_forward(m, d);

    simplechimera::KilowattChain controller(amplitude, 0.01, m->opt.timestep, 0.1, 0.05, 0.95, true);
    controller.reset();

    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to init GLFW\n");
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    GLFWwindow* win = glfwCreateWindow(1200, 900, "SimpleChimera (MuJoCo)", nullptr, nullptr);
    if (!win) {
        glfwTerminate();
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    glfwMakeContextCurrent(win);
    glfwShowWindow(win);
    glfwFocusWindow(win);
    glfwSwapInterval(1);
    std::fprintf(stderr, "MuJoCo viewer opened. Close the window or press Ctrl+C to stop.\n");

    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;
    mjrContext con;
    mjv_defaultScene(&scn);
    mjv_makeScene(m, &scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    double state[STATE_DIM];
    double action[ACTION_DIM];
    int step = 0;
    double totalReward = 0;

    if (testMotion)
        std::fprintf(stderr, "Test motion mode: applying strong oscillating torque to all joints (ignore Kilowatt).\n");

    while (!glfwWindowShouldClose(win) && step < maxSteps) {
        if (testMotion) {
            for (int i = 0; i < 18; i++)
                d->ctrl[i] = 0.08 * std::sin(step * 0.15 + i * 0.4);
        } else {
            mj_state_to_balance(m, d, state);
            controller.step(state, action);
            balance_action_to_mj(action, d);
        }
        mj_step(m, d);
        step++;

        mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        mjrRect rect = {0, 0, w, h};
        mjr_render(rect, &scn, &con);
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwMakeContextCurrent(win);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(win);
    glfwTerminate();
    mj_deleteData(d);
    mj_deleteModel(m);

    std::printf("Steps: %d  Total reward: (not computed in this viewer)\n", step);
    return 0;
}
