#include <nanobind/nanobind.h>
#include "ompl/control/StatePropagator.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_StatePropagator(nb::module_ &m)
{
    nb::class_<oc::StatePropagator>(m, "StatePropagator")
        .def("propagate", &oc::StatePropagator::propagate, nb::arg("state"), nb::arg("control"),
             nb::arg("duration"), nb::arg("result"))
        .def("canPropagateBackward", &oc::StatePropagator::canPropagateBackward)
        .def("steer", &oc::StatePropagator::steer, nb::arg("from"), nb::arg("to"), nb::arg("result"))
        .def("canSteer", &oc::StatePropagator::canSteer);
}
