/**
 * Python bindings for KilowattChain. Build with: cmake -Dpybind11_DIR=... ..
 * Then from Python: from kilowatt_cpp import KilowattChain; c = KilowattChain(0.5); c.reset(); c.step(state, action)
 */

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include "kilowatt_chain.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kilowatt_cpp, m) {
    py::class_<simplechimera::KilowattChain>(m, "KilowattChain")
        .def(py::init<double, double, double, double, double, double, bool>(),
             py::arg("amplitude") = 0.5,
             py::arg("dt_snn") = 0.01,
             py::arg("time_step") = 0.002,
             py::arg("k_couple") = 0.1,
             py::arg("tau") = 0.05,
             py::arg("I_bias") = 0.95,
             py::arg("use_phase_offset") = true)
        .def("reset", &simplechimera::KilowattChain::reset)
        .def("step",
             [](simplechimera::KilowattChain& self, py::array_t<double> state, py::array_t<double> action) {
                 py::buffer_info sb = state.request(), ab = action.request();
                 if (sb.ndim != 1 || ab.ndim != 1 ||
                     sb.shape[0] != simplechimera::STATE_DIM || ab.shape[0] != simplechimera::ACTION_DIM)
                     throw std::runtime_error("state must be (40,), action must be (18,)");
                 self.step(static_cast<const double*>(sb.ptr), static_cast<double*>(ab.ptr));
             },
             py::arg("state"), py::arg("action"));
    m.attr("STATE_DIM") = simplechimera::STATE_DIM;
    m.attr("ACTION_DIM") = simplechimera::ACTION_DIM;
}
