#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/SE2BeliefStateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_SE2BeliefStateSpace(nb::module_ &m)
{
    nb::class_<ob::SE2BeliefStateSpace::StateType, ob::State>(m, "SE2BeliefStateType")
        .def("getX", &ob::SE2BeliefStateSpace::StateType::getX)
        .def("getY", &ob::SE2BeliefStateSpace::StateType::getY)
        .def("getYaw", &ob::SE2BeliefStateSpace::StateType::getYaw)
        .def("setX", &ob::SE2BeliefStateSpace::StateType::setX, nb::arg("x"))
        .def("setY", &ob::SE2BeliefStateSpace::StateType::setY, nb::arg("y"))
        .def("setXY", &ob::SE2BeliefStateSpace::StateType::setXY, nb::arg("x"), nb::arg("y"))
        .def("setYaw", &ob::SE2BeliefStateSpace::StateType::setYaw, nb::arg("yaw"))
        .def("getCovariance", &ob::SE2BeliefStateSpace::StateType::getCovariance, nb::arg("i"))
        .def("setCovariance", &ob::SE2BeliefStateSpace::StateType::setCovariance, nb::arg("i"),
             nb::arg("value"));

    nb::class_<ob::SE2BeliefStateSpace, ob::CompoundStateSpace>(m, "SE2BeliefStateSpace")
        .def(nb::init<float>(), nb::arg("scale") = 1e-4f)
        .def("setBounds", &ob::SE2BeliefStateSpace::setBounds, nb::arg("bounds"))
        .def("getBounds", &ob::SE2BeliefStateSpace::getBounds, nb::rv_policy::reference_internal)
        .def("allocState", &ob::SE2BeliefStateSpace::allocState, nb::rv_policy::take_ownership)
        .def("freeState", &ob::SE2BeliefStateSpace::freeState, nb::arg("state"))
        .def("distance", &ob::SE2BeliefStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("wassersteinDistance", &ob::SE2BeliefStateSpace::wassersteinDistance, nb::arg("state1"),
             nb::arg("state2"))
        .def("registerProjections", &ob::SE2BeliefStateSpace::registerProjections);
}
