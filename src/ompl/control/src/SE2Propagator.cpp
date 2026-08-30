/**
 * Authors: Anonymous
 * Date: 2025-07-21
 */

#include "ompl/control/SE2Propagator.h"
#include <ompl/base/spaces/CovarianceStateSpace.h>
#include <ompl/control/LieGroup.h>

namespace ob = ompl::base;
namespace oc = ompl::control;


Eigen::Matrix3d toMatrix(const std::vector<double> &pose)
{
    Eigen::Matrix3d matrix;
    matrix << cos(pose[2]), -sin(pose[2]), pose[0],
              sin(pose[2]), cos(pose[2]), pose[1],
              0, 0, 1;
    return matrix;
}

std::vector<double> toVector(const Eigen::Matrix3d &matrix)
{
    std::vector<double> pose = {matrix(0, 2), matrix(1, 2), atan2(matrix(1, 0), matrix(0, 0))};
    return pose;
}

void oc::SE2Propagator::propagateSE2(const ob::State *state, 
                                     const ob::State *delta,
                                     ob::State *result)
{
    const auto *beliefState = state->as<ob::SE2StateSpace::StateType>();
    const auto *deltaState = delta->as<ob::SE2StateSpace::StateType>();
    auto *resultState = result->as<ob::SE2StateSpace::StateType>();

    // SE2 state propagation
    double x = beliefState->getX();
    double y = beliefState->getY();
    double theta = beliefState->getYaw();

    double deltaX = deltaState->getX();
    double deltaY = deltaState->getY();
    double deltaTheta = deltaState->getYaw();

    Eigen::Matrix3d initialPose = toMatrix({x, y, theta});
    Eigen::Matrix3d deltaPose = toMatrix({deltaX, deltaY, deltaTheta});
    Eigen::Matrix3d finalPose = initialPose * deltaPose;

    // Update result
    std::vector<double> finalVector = toVector(finalPose);
    resultState->setX(finalVector[0]);
    resultState->setY(finalVector[1]);
    resultState->setYaw(finalVector[2]);
}

void oc::SE2BeliefPropagator::propagateSE2Belief(const ob::State *state,
                                                 const ob::State *delta,
                                                 ob::State *result)
{
    const auto *beliefState = state->as<ob::SE2BeliefStateSpace::StateType>();
    const auto *deltaState = delta->as<ob::SE2BeliefStateSpace::StateType>();
    auto *resultState = result->as<ob::SE2BeliefStateSpace::StateType>();

    // SE2 state propagation
    double x = beliefState->getX();
    double y = beliefState->getY();
    double theta = beliefState->getYaw();

    double deltaX = deltaState->getX();
    double deltaY = deltaState->getY();
    double deltaTheta = deltaState->getYaw();

    Eigen::Matrix3d initialPose = toMatrix({x, y, theta});
    Eigen::Matrix3d deltaPose = toMatrix({deltaX, deltaY, deltaTheta});
    Eigen::Matrix3d finalPose = initialPose * deltaPose;

    std::vector<double> finalVector = toVector(finalPose);
    resultState->setX(finalVector[0]);
    resultState->setY(finalVector[1]);
    resultState->setYaw(finalVector[2]);

    // SE2 covariance propagation
    std::vector<double> beliefVector(6);
    Eigen::Matrix3d beliefMatrix;
    for (unsigned int i = 0; i < 6; ++i)
        beliefVector[i] = beliefState->getCovariance(i);
    beliefMatrix << beliefVector[0], beliefVector[1], beliefVector[2],
                    beliefVector[1], beliefVector[3], beliefVector[4],
                    beliefVector[2], beliefVector[4], beliefVector[5];

    std::vector<double> deltaVector(6);
    Eigen::Matrix3d deltaCov;
    for (unsigned int i = 0; i < 6; ++i)
        deltaVector[i] = deltaState->getCovariance(i);
    deltaCov << deltaVector[0], deltaVector[1], deltaVector[2],
                deltaVector[1], deltaVector[3], deltaVector[4],
                deltaVector[2], deltaVector[4], deltaVector[5];

    // Lie Algebebra
    std::vector<double> deltaTangent = oc::logSE2(deltaPose);
    // Eigen::Matrix3d jacR = oc::rightJacobianSE2(deltaTangent);
    for (unsigned int i = 0; i < deltaTangent.size(); ++i)
        deltaTangent[i] = -deltaTangent[i];
    Eigen::Matrix3d expDelta = oc::expSE2(deltaTangent);
    Eigen::Matrix3d adj = oc::adjointSE2(expDelta);

    Eigen::Matrix3d newCov = (adj * beliefMatrix * adj.transpose()) +
                             (deltaCov);
                            //  (jacR * deltaCov * jacR.transpose());
    resultState->setCovariance(0, newCov(0, 0));
    resultState->setCovariance(1, newCov(0, 1));
    resultState->setCovariance(2, newCov(0, 2));
    resultState->setCovariance(3, newCov(1, 1));
    resultState->setCovariance(4, newCov(1, 2));
    resultState->setCovariance(5, newCov(2, 2));
}
