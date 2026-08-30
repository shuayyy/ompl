/**
 * Authors: Anonymous
 * Date: 2025-07-01
 */

#ifndef OMPL_BASE_SPACES_COVARIANCE_STATE_SPACE_H
#define OMPL_BASE_SPACES_COVARIANCE_STATE_SPACE_H

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <Eigen/Dense>

// Define the state space type enum
#define COV_STATE_SPACE 100

namespace ompl
{
    namespace base
    {
        /** \brief State space sampler for covariance space */
        class CovarianceStateSampler : public StateSampler
        {
            public:
                /** \brief Constructor */
                CovarianceStateSampler(const StateSpace *space, int stateDim, float scale = 1e-4);

                void sampleUniform(State *state) override;
                void sampleUniformNear(State *state, const State *near, double distance) override;
                void sampleGaussian(State *state, const State *mean, double stdDev) override;

                protected:
                unsigned int stateDim_;
                float scale_;
        };

        class CovarianceStateSpace : public RealVectorStateSpace
        {
            public:
                CovarianceStateSpace(unsigned int stateDim, float sampleScale = 1e-4);
                ~CovarianceStateSpace() = default;

                double distance(const State *state1, const State *state2) const override;

                double wassersteinDistance(const Eigen::MatrixXd& cov1, const Eigen::MatrixXd& cov2) const;

                void interpolate(const State *from, const State *to, double t, State *state) const override;

                /**
                 * @brief Ensure a covariance matrix is symmetric positive definite (SPD).
                 * If the matrix is not SPD, this function will emit an OMPL warning and
                 * project it to an SPD matrix (first via diagonal jitter; if that fails,
                 * via eigenvalue clamping).
                 *
                 * @param cov The covariance matrix to validate/repair (modified in-place).
                 * @return true if the matrix was already SPD, false if it was repaired.
                 */
                bool ensureCovariance(Eigen::MatrixXd& cov) const;

                // Utility functions for covariance <-> vector conversion
                /**
                 * @brief Extracts the unique elements (upper triangle) of a 
                 * symmetric matrix into a stateDim_-element vector.
                 *
                 * @param cov The input stateDim_-element vector.
                 * @return A stateDim_-element vector containing [c00, c01, c02, c11, c12, c22].
                 */
                Eigen::VectorXd cov2vec(const Eigen::MatrixXd& cov) const;
                /**
                 * @brief Constructs a stateDim_-element symmetric matrix from a stateDim_-element vector
                 * containing its unique upper-triangular elements.
                 *
                 * @param vec The stateDim_-element input vector [c00, c01, c02, c11, c12, c22].
                 * @return The reconstructed stateDim_-element symmetric matrix.
                 */
                Eigen::MatrixXd vec2cov(const Eigen::VectorXd& vec) const;

                void registerProjections() override;

            protected:
                unsigned int stateDim_;
                unsigned int vecDim_;
        };
    }
}

#endif