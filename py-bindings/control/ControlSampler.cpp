#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/trampoline.h>

#include "ompl/control/ControlSampler.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"

#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_ControlSampler(nb::module_ &m)
{
    struct PyControlSampler : oc::ControlSampler
    {
        NB_TRAMPOLINE(oc::ControlSampler, 5);

        PyControlSampler(const oc::ControlSpace *space) : oc::ControlSampler(space)
        {
        }

        void sample(oc::Control *control) override
        {
            NB_OVERRIDE_PURE(sample, control);
        }

        void sample(oc::Control *control, const ob::State *state) override
        {
            NB_OVERRIDE(sample, control, state);
        }

        void sampleNext(oc::Control *control, const oc::Control *previous) override
        {
            NB_OVERRIDE(sampleNext, control, previous);
        }

        void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State *state) override
        {
            NB_OVERRIDE(sampleNext, control, previous, state);
        }

        unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps) override
        {
            NB_OVERRIDE(sampleStepCount, minSteps, maxSteps);
        }
    };

    nb::class_<oc::ControlSampler, PyControlSampler /* <-- trampoline */>(m, "ControlSampler")
        .def(nb::init<const oc::ControlSpace *>(), nb::arg("space"), nb::keep_alive<1, 2>())
        .def("sample", nb::overload_cast<oc::Control *>(&oc::ControlSampler::sample), nb::arg("control"))
        .def("sample", nb::overload_cast<oc::Control *, const ob::State *>(&oc::ControlSampler::sample),
             nb::arg("control"), nb::arg("state"))
        .def("sampleNext", nb::overload_cast<oc::Control *, const oc::Control *>(&oc::ControlSampler::sampleNext),
             nb::arg("control"), nb::arg("previous"))
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *>(&oc::ControlSampler::sampleNext),
             nb::arg("control"), nb::arg("previous"), nb::arg("state"))
        .def("sampleStepCount", &oc::ControlSampler::sampleStepCount, nb::arg("minSteps"), nb::arg("maxSteps"));

    nb::class_<oc::CompoundControlSampler, oc::ControlSampler>(m, "CompoundControlSampler")
        .def(nb::init<const oc::ControlSpace *>(), nb::arg("space"),
             "Construct a CompoundControlSampler for the given control space")
        .def("addSampler", &oc::CompoundControlSampler::addSampler, nb::arg("sampler"))
        // Overloaded sample() methods:
        .def("sample", nb::overload_cast<oc::Control *>(&oc::CompoundControlSampler::sample), nb::arg("control"),
             "Sample a control uniformly")
        .def("sample", nb::overload_cast<oc::Control *, const ob::State *>(&oc::CompoundControlSampler::sample),
             nb::arg("control"), nb::arg("state"), "Sample a control uniformly near the given state")
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *>(&oc::CompoundControlSampler::sampleNext),
             nb::arg("control"), nb::arg("previous"), "Sample the next control based on the previous control")
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *>(
                 &oc::CompoundControlSampler::sampleNext),
             nb::arg("control"), nb::arg("previous"), nb::arg("state"),
             "Sample the next control based on the previous control and a given state");
}
