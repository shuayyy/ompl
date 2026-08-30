/**
 * Authors: Anonymous
 * Date: 2025-07-21
 */

#ifndef OMPL_CONTROL_SE2PROPAGATOR_H
#define OMPL_CONTROL_SE2PROPAGATOR_H

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE2BeliefStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/config.h>
#include <iostream>
#include <limits>
#include <boost/math/constants/constants.hpp>
#include <vector>
#include <Eigen/Dense>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace ompl
{
    namespace control
    {
        class SE2Pose {
        public:
            SE2Pose(double x = 0.0, double y = 0.0, double theta = 0.0) 
                : x_(x), y_(y), theta_(theta) {}
            
            // Create from vector [x, y, theta]
            SE2Pose(const std::vector<double>& pose) 
                : x_(pose[0]), y_(pose[1]), theta_(pose[2]) {}
            
            // Composition operator: this_pose @ other_pose
            SE2Pose operator*(const SE2Pose& other) const {
                double cos_t = cos(theta_);
                double sin_t = sin(theta_);
                
                return SE2Pose(
                    x_ + cos_t * other.x_ - sin_t * other.y_,
                    y_ + sin_t * other.x_ + cos_t * other.y_,
                    theta_ + other.theta_
                );
            }
            
            // Inverse pose
            SE2Pose inverse() const {
                double cos_t = cos(-theta_);
                double sin_t = sin(-theta_);
                
                return SE2Pose(
                    cos_t * (-x_) - sin_t * (-y_),
                    sin_t * (-x_) + cos_t * (-y_),
                    -theta_
                );
            }
            
            // Difference: other_pose - this_pose (in local frame)
            SE2Pose difference(const SE2Pose& other) const {
                return inverse() * other;
            }
            
            // To vector
            std::vector<double> toVector() const {
                return {x_, y_, theta_};
            }

            // Set the list of parameters
            void setParams(const std::vector<double>& params) {
                x_ = params[0];
                y_ = params[1];
                theta_ = params[2];
            }

            // Get the list of parameters
            std::vector<double> getParams() const {
                return {x_, y_, theta_};
            }

        private:
            double x_, y_, theta_;
        };

        class SE2Propagator
        {
            public:
                SE2Propagator() = default;

                SE2Propagator(SpaceInformation *si)
                {
                    (void)si;
                }

                SE2Propagator(const SpaceInformationPtr &si) 
                {
                    (void)si;
                }

                void propagateSE2(const ob::State *state,
                                  const ob::State *delta,
                                  ob::State *result);
        };

        class SE2BeliefPropagator
        {
            public:
                SE2BeliefPropagator() = default;

                SE2BeliefPropagator(SpaceInformation *si)
                {
                    (void)si;
                }

                SE2BeliefPropagator(const SpaceInformationPtr &si) 
                {
                    (void)si;
                }

                void propagateSE2Belief(const ob::State *state,
                                        const ob::State *delta,
                                        ob::State *result);
        };
    }
}

#endif
