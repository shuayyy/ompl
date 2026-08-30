#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/LieGroup.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::init_LieGroup(nb::module_ &m)
{
    m.def(
        "logSE2",
        [](const Eigen::Matrix3d &transform)
        {
            Eigen::Matrix3d t = transform;
            return oc::logSE2(t);
        },
        nb::arg("transform"));
    m.def("expSE2", &oc::expSE2, nb::arg("xi"));
    m.def(
        "adjointSE2",
        [](const Eigen::Matrix3d &transform)
        {
            Eigen::Matrix3d t = transform;
            return oc::adjointSE2(t);
        },
        nb::arg("transform"));
    m.def("rightJacobianSE2", &oc::rightJacobianSE2, nb::arg("xi"));
    m.def(
        "getSE2Vec",
        [](const Eigen::Matrix3d &transform)
        {
            Eigen::Matrix3d t = transform;
            return oc::getSE2Vec(t);
        },
        nb::arg("transform"));
    m.def("getTransform", &oc::getTransform, nb::arg("params"));
    m.def(
        "invTransformSE2",
        [](const Eigen::Matrix3d &transform)
        {
            Eigen::Matrix3d t = transform;
            return oc::invTransformSE2(t);
        },
        nb::arg("transform"));
}
