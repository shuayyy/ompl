/**
 * Authors: Anonymous
 * Date: 2025-07-07
 */

#ifndef OMPL_CONTROL_LIE_GROUP_H
#define OMPL_CONTROL_LIE_GROUP_H

#include <vector>
#include <Eigen/Dense>

namespace ompl
{
    namespace control
    {
        std::vector<double> logSE2(Eigen::Matrix3d &transform);

        Eigen::Matrix3d expSE2(const std::vector<double> &xi);

        Eigen::Matrix3d adjointSE2(Eigen::Matrix3d &transform);

        Eigen::Matrix3d rightJacobianSE2(const std::vector<double> &xi);

        std::vector<double> getSE2Vec(Eigen::Matrix3d &transform);

        Eigen::Matrix3d getTransform(const std::vector<double> &params);

        Eigen::Matrix3d invTransformSE2(Eigen::Matrix3d &transform);
    }
}

#endif