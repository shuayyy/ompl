/**
 * Authors: Anonymous
 * Date: 2025-07-07
 */

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE2BeliefStateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>

ompl::base::State *ompl::base::SE2BeliefStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::SE2BeliefStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

double ompl::base::SE2BeliefStateSpace::distance(const State *state1, const State *state2) const
{
    // Ignore covariance part for distance calculation
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    double dist = components_[0]->distance(cstate1->components[0], cstate2->components[0]);
    return dist;
}

double ompl::base::SE2BeliefStateSpace::wassersteinDistance(const State *state1, const State *state2) const
{
    // Regular compound distance, this will end up the same as Wasserstein Distance as it becomes
    // SE2 distance + Covariance distance (wasserstein distance with mean 0)
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    double dist = 0.0;
    for (unsigned int i = 0; i < componentCount_; ++i)
        dist += weights_[i] * components_[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

void ompl::base::SE2BeliefStateSpace::registerProjections()
{
    // Similar as SE2StateSpace projection, only project (x, y)
    class SE2BeliefDefaultProjection : public ProjectionEvaluator
    {
    public:
        SE2BeliefDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 2;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(2);
            bounds_ = space_->as<SE2BeliefStateSpace>()->getSubspace(0)->as<SE2StateSpace>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection = Eigen::Map<const Eigen::VectorXd>(
                state->as<SE2BeliefStateSpace::StateType>()->as<SE2StateSpace::StateType>(0)->as<RealVectorStateSpace::StateType>(0)->values, 2);
        }
    };

    registerDefaultProjection(std::make_shared<SE2BeliefDefaultProjection>(this));
}
