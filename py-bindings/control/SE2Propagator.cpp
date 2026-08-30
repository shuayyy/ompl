#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/SE2Propagator.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

void ompl::binding::control::init_SE2Propagator(nb::module_ &m)
{
    nb::class_<oc::SE2Pose>(m, "SE2Pose")
        .def(nb::init<double, double, double>(), nb::arg("x") = 0.0, nb::arg("y") = 0.0, nb::arg("theta") = 0.0)
        .def(nb::init<const std::vector<double> &>(), nb::arg("pose"))
        .def("__mul__", &oc::SE2Pose::operator*, nb::arg("other"))
        .def("inverse", &oc::SE2Pose::inverse)
        .def("difference", &oc::SE2Pose::difference, nb::arg("other"))
        .def("toVector", &oc::SE2Pose::toVector)
        .def("setParams", &oc::SE2Pose::setParams, nb::arg("params"))
        .def("getParams", &oc::SE2Pose::getParams);

    nb::class_<oc::SE2Propagator>(m, "SE2Propagator")
        .def(nb::init<>())
        .def(nb::init<oc::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("propagateSE2", &oc::SE2Propagator::propagateSE2, nb::arg("state"), nb::arg("delta"),
             nb::arg("result"));

    nb::class_<oc::SE2BeliefPropagator>(m, "SE2BeliefPropagator")
        .def(nb::init<>())
        .def(nb::init<oc::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("propagateSE2Belief", &oc::SE2BeliefPropagator::propagateSE2Belief, nb::arg("state"),
             nb::arg("delta"), nb::arg("result"));
}
