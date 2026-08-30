#include <limits>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalLazySamples(nb::module_ &m)
{
    struct PyGoalLazySamples : ob::GoalLazySamples
    {
        NB_TRAMPOLINE(ob::GoalLazySamples, 9);

        void sampleGoal(ob::State *st) const override
        {
            NB_OVERRIDE(sampleGoal, st);
        }

        double distanceGoal(const ob::State *st) const override
        {
            NB_OVERRIDE(distanceGoal, st);
        }

        void addState(const ob::State *st) override
        {
            NB_OVERRIDE(addState, st);
        }

        bool couldSample() const override
        {
            NB_OVERRIDE(couldSample);
        }

        bool hasStates() const override
        {
            NB_OVERRIDE(hasStates);
        }

        const ob::State *getState(unsigned int index) const override
        {
            NB_OVERRIDE(getState, index);
        }

        std::size_t getStateCount() const override
        {
            NB_OVERRIDE(getStateCount);
        }

        void clear() override
        {
            NB_OVERRIDE(clear);
        }

        unsigned int maxSampleCount() const override
        {
            NB_OVERRIDE(maxSampleCount);
        }
    };

    nb::class_<ob::GoalLazySamples, ob::GoalStates, PyGoalLazySamples /* <-- trampoline */>(m, "GoalLazySamples")
        .def(nb::init<const ob::SpaceInformationPtr &, ob::GoalSamplingFn, bool, double>())
        .def("sampleGoal", &ob::GoalLazySamples::sampleGoal, nb::arg("state"))
        .def("distanceGoal", &ob::GoalLazySamples::distanceGoal, nb::arg("state"))
        .def("addState", &ob::GoalLazySamples::addState, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalLazySamples::maxSampleCount)
        .def("startSampling", &ob::GoalLazySamples::startSampling)
        .def("stopSampling", &ob::GoalLazySamples::stopSampling)
        .def("isSampling", &ob::GoalLazySamples::isSampling)
        .def("samplingAttemptsCount", &ob::GoalLazySamples::samplingAttemptsCount,
             "Total calls to the sampler function so far.")
        .def("setMinNewSampleDistance", &ob::GoalLazySamples::setMinNewSampleDistance, nb::arg("dist"),
             "Require new samples to be at least this far from all existing ones.")
        .def("getMinNewSampleDistance", &ob::GoalLazySamples::getMinNewSampleDistance)
        .def("setNewStateCallback", &ob::GoalLazySamples::setNewStateCallback, nb::arg("callback"))
        .def("addStateIfDifferent", &ob::GoalLazySamples::addStateIfDifferent, nb::arg("state"), nb::arg("minDistance"))
        .def("couldSample", &ob::GoalLazySamples::couldSample)
        .def("hasStates", &ob::GoalLazySamples::hasStates)
        .def("getState", &ob::GoalLazySamples::getState, nb::arg("index"), nb::rv_policy::reference_internal)
        .def("getStateCount", &ob::GoalLazySamples::getStateCount)
        .def("clear", &ob::GoalLazySamples::clear);
}
