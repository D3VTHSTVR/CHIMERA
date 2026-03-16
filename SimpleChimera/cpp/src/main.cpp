/**
 * Standalone Kilowatt controller: read state (40 floats) from stdin (binary),
 * write action (18 floats) to stdout (binary). For use with Python env via pipe
 * or subprocess. One line: 40 * sizeof(double) bytes in, 18 * sizeof(double) bytes out per step.
 *
 * Usage: ./kilowatt_standalone [amplitude]
 *   amplitude in (0,1], default 0.5
 */

#include "kilowatt_chain.hpp"
#include <cstdio>
#include <cstdlib>
#include <vector>

int main(int argc, char** argv) {
    double amplitude = 0.5;
    if (argc >= 2)
        amplitude = std::atof(argv[1]);
    if (amplitude <= 0.0 || amplitude > 1.0)
        amplitude = 0.5;

    simplechimera::KilowattChain chain(amplitude, 0.01, 0.002, 0.1, 0.05, 0.95, true);
    std::vector<double> state(simplechimera::STATE_DIM, 0.0);
    std::vector<double> action(simplechimera::ACTION_DIM, 0.0);

    chain.reset();
    while (std::fread(state.data(), sizeof(double), simplechimera::STATE_DIM, stdin) ==
           static_cast<size_t>(simplechimera::STATE_DIM)) {
        chain.step(state.data(), action.data());
        if (std::fwrite(action.data(), sizeof(double), simplechimera::ACTION_DIM, stdout) !=
            static_cast<size_t>(simplechimera::ACTION_DIM))
            break;
    }
    return 0;
}
