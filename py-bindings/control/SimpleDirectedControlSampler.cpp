#include <nanobind/nanobind.h>
#include "ompl/control/SimpleDirectedControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_SimpleDirectedControlSampler(nb::module_ &m)
{
    nb::class_<oc::SimpleDirectedControlSampler, oc::DirectedControlSampler>(m, "SimpleDirectedControlSampler")
        .def(nb::init<const oc::SpaceInformation *, unsigned int>(), nb::arg("si"), nb::arg("k") = 1)
        .def("getNumControlSamples", &oc::SimpleDirectedControlSampler::getNumControlSamples)
        .def("setNumControlSamples", &oc::SimpleDirectedControlSampler::setNumControlSamples, nb::arg("numSamples"))
        .def("sampleTo",
             nb::overload_cast<oc::Control *, const ob::State *, ob::State *>(
                 &oc::SimpleDirectedControlSampler::sampleTo),
             nb::arg("control"), nb::arg("source"), nb::arg("dest"))
        .def("sampleTo",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *, ob::State *>(
                 &oc::SimpleDirectedControlSampler::sampleTo),
             nb::arg("control"), nb::arg("previous"), nb::arg("source"), nb::arg("dest"));
}
