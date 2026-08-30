#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>

#include <sstream>
#include "ompl/base/goals/GoalSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalSpace(nb::module_ &m)
{
    struct PyGoalSpace : ob::GoalSpace
    {
        NB_TRAMPOLINE(ob::GoalSpace, 3);

        void sampleGoal(ob::State *st) const override
        {
            NB_OVERRIDE(sampleGoal, st);
        }

        unsigned int maxSampleCount() const override
        {
            NB_OVERRIDE(maxSampleCount);
        }

        double distanceGoal(const ob::State *st) const override
        {
            NB_OVERRIDE(distanceGoal, st);
        }
    };

    nb::class_<ob::GoalSpace, ob::GoalSampleableRegion, PyGoalSpace /* <-- trampoline */>(m, "GoalSpace")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("sampleGoal", &ob::GoalSpace::sampleGoal, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalSpace::maxSampleCount)
        .def("distanceGoal", &ob::GoalSpace::distanceGoal, nb::arg("state"))
        .def("print", [](const ob::GoalSpace &self) { self.print(std::cout); })
        .def("__repr__",
             [](const ob::GoalSpace &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
        .def("setSpace", &ob::GoalSpace::setSpace, nb::arg("si"))
        .def("getSpace", &ob::GoalSpace::getSpace, nb::rv_policy::reference_internal);
}
