#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/PlannerData.h"
#include "ompl/base/PlannerData.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_PlannerData(nb::module_ &m)
{
    nb::class_<oc::PlannerDataEdgeControl, ob::PlannerDataEdge>(m, "PlannerDataEdgeControl")
        .def(nb::init<const oc::Control *, double>(), nb::arg("control"), nb::arg("duration"))
        .def(nb::init<const oc::PlannerDataEdgeControl &>(), nb::arg("rhs"))
        .def("clone", &oc::PlannerDataEdgeControl::clone)
        .def("getControl", &oc::PlannerDataEdgeControl::getControl, nb::rv_policy::reference_internal)
        .def("getDuration", &oc::PlannerDataEdgeControl::getDuration);

    nb::class_<oc::PlannerData, ob::PlannerData>(m, "PlannerData")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("removeVertex", nb::overload_cast<const ob::PlannerDataVertex &>(&oc::PlannerData::removeVertex),
             nb::arg("st"))
        .def("removeVertex", nb::overload_cast<unsigned int>(&oc::PlannerData::removeVertex), nb::arg("vIndex"))
        .def("removeEdge", nb::overload_cast<unsigned int, unsigned int>(&oc::PlannerData::removeEdge), nb::arg("v1"),
             nb::arg("v2"))
        .def("removeEdge",
             nb::overload_cast<const ob::PlannerDataVertex &, const ob::PlannerDataVertex &>(
                 &oc::PlannerData::removeEdge),
             nb::arg("v1"), nb::arg("v2"))
        .def("clear", &oc::PlannerData::clear)
        .def("decoupleFromPlanner", &oc::PlannerData::decoupleFromPlanner)
        .def("getSpaceInformation", &oc::PlannerData::getSpaceInformation, nb::rv_policy::reference_internal)
        .def("hasControls", &oc::PlannerData::hasControls);
}
