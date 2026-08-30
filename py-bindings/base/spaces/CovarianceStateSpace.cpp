#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/CovarianceStateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_CovarianceStateSpace(nb::module_ &m)
{
    m.attr("COV_STATE_SPACE") = nb::cast(COV_STATE_SPACE);

    nb::class_<ob::CovarianceStateSampler, ob::StateSampler>(m, "CovarianceStateSampler")
        .def(nb::init<const ob::StateSpace *, int, float>(), nb::arg("space"), nb::arg("stateDim"),
             nb::arg("scale") = 1e-4f)
        .def("sampleUniform", &ob::CovarianceStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::CovarianceStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::CovarianceStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::CovarianceStateSpace, ob::RealVectorStateSpace>(m, "CovarianceStateSpace")
        .def(nb::init<unsigned int, float>(), nb::arg("stateDim"), nb::arg("sampleScale") = 1e-4f)
        .def("distance", &ob::CovarianceStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("wassersteinDistance", &ob::CovarianceStateSpace::wassersteinDistance, nb::arg("cov1"),
             nb::arg("cov2"))
        .def(
            "ensureCovariance",
            [](const ob::CovarianceStateSpace &space, const Eigen::MatrixXd &cov)
            {
                Eigen::MatrixXd repaired = cov;
                bool alreadySpd = space.ensureCovariance(repaired);
                return std::make_pair(alreadySpd, repaired);
            },
            nb::arg("cov"))
        .def("interpolate", &ob::CovarianceStateSpace::interpolate, nb::arg("from"), nb::arg("to"), nb::arg("t"),
             nb::arg("state"))
        .def("cov2vec", &ob::CovarianceStateSpace::cov2vec, nb::arg("cov"))
        .def("vec2cov", &ob::CovarianceStateSpace::vec2cov, nb::arg("vec"))
        .def("registerProjections", &ob::CovarianceStateSpace::registerProjections);
}
