#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/STRRTstar.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

void ompl::binding::geometric::initPlannersRrt_STRRTstar(nb::module_ &m)
{
    // TAG [og::STRRTstar][Planner]
    nb::class_<STRRTstar, Planner>(m, "STRRTstar")
        .def(nb::init<const SpaceInformationPtr &>(), nb::arg("si"))
        .def("setRange", &STRRTstar::setRange, nb::arg("distance"))
        .def("getRange", &STRRTstar::getRange)
        .def("getOptimumApproxFactor", &STRRTstar::getOptimumApproxFactor)
        .def("setOptimumApproxFactor", &STRRTstar::setOptimumApproxFactor,
             nb::arg("optimumApproxFactor"))
        .def("setRewiringToOff", &STRRTstar::setRewiringToOff)
        .def("setRewiringToRadius", &STRRTstar::setRewiringToRadius)
        .def("setRewiringToKNearest", &STRRTstar::setRewiringToKNearest)
        .def("getRewireFactor", &STRRTstar::getRewireFactor)
        .def("setRewireFactor", &STRRTstar::setRewireFactor, nb::arg("v"))
        .def("getBatchSize", &STRRTstar::getBatchSize)
        .def("setBatchSize", &STRRTstar::setBatchSize, nb::arg("v"))
        .def("setTimeBoundFactorIncrease", &STRRTstar::setTimeBoundFactorIncrease,
             nb::arg("f"))
        .def("setInitialTimeBoundFactor", &STRRTstar::setInitialTimeBoundFactor,
             nb::arg("f"))
        .def("setSampleUniformForUnboundedTime",
             &STRRTstar::setSampleUniformForUnboundedTime, nb::arg("uniform"))
        .def("setup", &STRRTstar::setup)
        .def("clear", &STRRTstar::clear)
        .def("solve",
             [](STRRTstar &self, nb::object what)
             {
                 if (nb::isinstance<PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def("getPlannerData", &STRRTstar::getPlannerData, nb::arg("data"));
}
