/**
 * Authors: Anonymous
 * Date: 2025-07-0
 */

#ifndef SE2_BELIEF_STATE_SPACE_
#define SE2_BELIEF_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/CovarianceStateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief A state space representing SE2Belief */
        class SE2BeliefStateSpace : public CompoundStateSpace
        {
        public:
            /** \brief A state in SE2Belief: (x, y, yaw, covariance) */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                /** \brief Get the X component of the state */
                double getX() const
                {
                    return as<SE2StateSpace::StateType>(0)->getX();
                }

                /** \brief Get the Y component of the state */
                double getY() const
                {
                    return as<SE2StateSpace::StateType>(0)->getY();
                }

                /** \brief Get the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                double getYaw() const
                {
                    return as<SE2StateSpace::StateType>(0)->getYaw();
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<SE2StateSpace::StateType>(0)->setX(x);
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<SE2StateSpace::StateType>(0)->setY(y);
                }

                /** \brief Set the X and Y components of the state */
                void setXY(double x, double y)
                {
                    setX(x);
                    setY(y);
                }

                /** \brief Set the Yaw components of the state */
                void setYaw(double yaw)
                {
                    as<SE2StateSpace::StateType>(0)->setYaw(yaw);
                }

                /** \brief Get the covariance component of the state */
                double getCovariance(unsigned int i) const
                {
                    if (i >= 6)
                        throw std::invalid_argument("Covariance only has 6 elements.");
                    return as<CovarianceStateSpace::StateType>(1)->values[i];
                }

                /** \brief Set the covariance component of the state */
                void setCovariance(unsigned int i, double value)
                {
                    if (i >= 6)
                        throw std::invalid_argument("Covariance only has 6 elements.");
                    as<CovarianceStateSpace::StateType>(1)->values[i] = value;
                }
            };

            SE2BeliefStateSpace(float scale = 1e-4)
            {
                setName("SE2Belief");
                type_ = STATE_SPACE_SE2;
                addSubspace(std::make_shared<SE2StateSpace>(), 1.0);
                addSubspace(std::make_shared<CovarianceStateSpace>(3, scale), 1.0);
                lock();
            }

            ~SE2BeliefStateSpace() override = default;

            /** \copydoc RealVectorStateSpace::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<SE2StateSpace>(0)->setBounds(bounds);
                as<CovarianceStateSpace>(1)->setBounds(-10.0, 10.0);
            }

            /** \copydoc RealVectorStateSpace::getBounds() */
            const RealVectorBounds &getBounds() const
            {
                return as<SE2StateSpace>(0)->getBounds();
            }

            State *allocState() const override;
            void freeState(State *state) const override;
            double distance(const State *state1, const State *state2) const override;
            double wassersteinDistance(const State *state1, const State *state2) const;

            void registerProjections() override;
        };
    }
}

#endif
