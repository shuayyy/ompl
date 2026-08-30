/**
 * Authors: Anonymous
 * Date: 2025-07-01
 */

#include "ompl/base/spaces/CovarianceStateSpace.h"
#include "ompl/base/spaces/RealVectorStateProjections.h"
#include <Eigen/Dense>

ompl::base::CovarianceStateSampler::CovarianceStateSampler(const StateSpace *space, int stateDim, float scale)
    : StateSampler(space), stateDim_(stateDim), scale_(scale)
{
}

// TODO: Sampling for Covariance matrix is under developed
// and it is not really used in practice yet
void ompl::base::CovarianceStateSampler::sampleUniform(State *state)
{
    // Use a simple fixed covariance matrix
    double* val = static_cast<base::CovarianceStateSpace::StateType*>(state)->values;
    int k = 0;
    for (int i = 0; i < stateDim_; ++i)
        for (int j = i; j < stateDim_; ++j)
            if (i == j)
                val[k++] = scale_;  // Diagonal elements
            else
                val[k++] = 0.0;
}

void ompl::base::CovarianceStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    // TODO: For now
    // Just sample uniformly
    sampleUniform(state);
}

void ompl::base::CovarianceStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    // TODO: Mean is ignored for now

    // R ~ N(0, scale) ^ dxd
    Eigen::MatrixXd r(stateDim_, stateDim_);
    for (int i = 0; i < stateDim_; ++i)
        for (int j = 0; j < stateDim_; ++j)
            r(i, j) = rng_.gaussian(0.0, stdDev);

    // Cov = R R^T (PSD) then add tiny diag for PSD
    Eigen::MatrixXd cov = r * r.transpose();
    cov.diagonal().array() += 1e-6;   // ensure PSD

    // Convert the state to the vec vector
    double* val = static_cast<CovarianceStateSpace::StateType*>(state)->values;
    // loop to pick upper triangle
    int k = 0;
    for (int i = 0; i < stateDim_; ++i)
        for (int j = i; j < stateDim_; ++j)
            val[k++] = cov(i, j);
}

ompl::base::CovarianceStateSpace::CovarianceStateSpace(unsigned int stateDim, float sampleScale)
    : RealVectorStateSpace(stateDim * (stateDim + 1) / 2)
{
    // Set the state dimension to the number of elements in the covariance matrix
    stateDim_ = stateDim;
    vecDim_ = stateDim * (stateDim + 1) / 2;
    type_ = COV_STATE_SPACE;
    setName("CovarianceStateSpace");
    dimensionNames_.resize(vecDim_, "");
    setStateSamplerAllocator([stateDim, sampleScale](const ompl::base::StateSpace *space) {
        return std::make_shared<ompl::base::CovarianceStateSampler>(space, stateDim, sampleScale);
    });

    // Set the bounds to the maximum value of the covariance matrix
    // But bounds should not be really used for covariance matrices
    ompl::base::RealVectorBounds beliefBounds(vecDim_);
    beliefBounds.setLow(-100.0);
    beliefBounds.setHigh(100.0);
    setBounds(beliefBounds);
}

bool ompl::base::CovarianceStateSpace::ensureCovariance(Eigen::MatrixXd &cov) const
{
    // Basic sanity: correct shape
    if (cov.rows() != static_cast<int>(stateDim_) || cov.cols() != static_cast<int>(stateDim_))
    {
        OMPL_WARN("CovarianceStateSpace::ensureCovariance: wrong shape (%dx%d), expected %ux%u. Forcing identity.",
                  cov.rows(), cov.cols(), stateDim_, stateDim_);
        cov = Eigen::MatrixXd::Identity(stateDim_, stateDim_) * 1e-6;
        return false;
    }

    // Fast path: already SPD (after symmetrize)
    Eigen::MatrixXd sym = 0.5 * (cov + cov.transpose());
    Eigen::LLT<Eigen::MatrixXd> llt(sym);
    if (llt.info() == Eigen::Success)
    {
        cov = sym;
        return true;
    }

    // Not SPD -> warn and repair
    OMPL_WARN("Non-SPD covariance detected; projecting to SPD.");

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(sym);
    Eigen::VectorXd d = es.eigenvalues();
    Eigen::MatrixXd V = es.eigenvectors();
    d = d.cwiseMax(1e-9);
    cov = V * d.asDiagonal() * V.transpose();
    return false;
}

double ompl::base::CovarianceStateSpace::distance(const State *state1, const State *state2) const
{
    // Get the covariance matrices from the states
    Eigen::Map<const Eigen::VectorXd> vec1(state1->as<CovarianceStateSpace::StateType>()->values, vecDim_);
    Eigen::Map<const Eigen::VectorXd> vec2(state2->as<CovarianceStateSpace::StateType>()->values, vecDim_);
    Eigen::MatrixXd cov1 = vec2cov(vec1);
    Eigen::MatrixXd cov2 = vec2cov(vec2);

    // Compute the distance between the covariance matrices
    return wassersteinDistance(cov1, cov2);
}

double ompl::base::CovarianceStateSpace::wassersteinDistance(const Eigen::MatrixXd& cov1, const Eigen::MatrixXd& cov2) const
{
    // Make local copies we can repair if needed.
    Eigen::MatrixXd c1 = cov1;
    Eigen::MatrixXd c2 = cov2;
    bool isCov1 = ensureCovariance(c1);
    bool isCov2 = ensureCovariance(c2);
    if (!isCov1)
        OMPL_WARN("CovarianceStateSpace::wassersteinDistance: input covariance 1 were not SPD.");  
    if (!isCov2)
        OMPL_WARN("CovarianceStateSpace::wassersteinDistance: input covariance 2 were not SPD.");

    // Compute the distance between the covariance matrices
    // General formula
    // cov3 = cov1Sqrt * cov2 * cov1Sqrt;
    // (cov1 + cov2 - 2 * cov3Sqrt).trace();
    // Here is an optimized version
    // cov1.trace() + cov2.trace() - 2 * cov3.eigenvalues().sqrt().sum();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es1(c1);
    Eigen::MatrixXd cov1Sqrt = es1.eigenvectors() * es1.eigenvalues().cwiseSqrt().asDiagonal() * es1.eigenvectors().transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es3(cov1Sqrt * c2 * cov1Sqrt);
    double sqrtTr = es3.eigenvalues().array().sqrt().sum();

    return c1.trace() + c2.trace() - 2.0 * sqrtTr;
}

void ompl::base::CovarianceStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
{
    // Map vectors
    Eigen::Map<const Eigen::VectorXd> v1(from->as<CovarianceStateSpace::StateType>()->values, vecDim_);
    Eigen::Map<const Eigen::VectorXd> v2(to->as<CovarianceStateSpace::StateType>()->values, vecDim_);
    Eigen::Map<Eigen::VectorXd> vo(state->as<CovarianceStateSpace::StateType>()->values, vecDim_);

    // Convert to covariance matrices
    Eigen::MatrixXd c1 = vec2cov(v1);
    Eigen::MatrixXd c2 = vec2cov(v2);
    bool isCov1 = ensureCovariance(c1);
    bool isCov2 = ensureCovariance(c2);
    if (!isCov1)
        OMPL_WARN("CovarianceStateSpace::interpolate: 'from' covariance was not SPD.");
    if (!isCov2)
        OMPL_WARN("CovarianceStateSpace::interpolate: 'to' covariance was not SPD.");

    // Cholesky
    Eigen::LLT<Eigen::MatrixXd> llt1(c1);
    Eigen::LLT<Eigen::MatrixXd> llt2(c2);
    Eigen::MatrixXd L1 = llt1.matrixL();
    Eigen::MatrixXd L2 = llt2.matrixL();

    // Interpolate L
    Eigen::MatrixXd L = (1.0 - t) * L1 + t * L2;

    // Rebuild covariance (SPD)
    Eigen::MatrixXd Cout = L * L.transpose();
    Cout.diagonal().array() += 1e-9;

    // Back to vector
    Eigen::VectorXd vout = cov2vec(Cout);
    vo = vout;
}

Eigen::VectorXd ompl::base::CovarianceStateSpace::cov2vec(const Eigen::MatrixXd& cov) const
{
    assert (cov.rows() == cov.cols() && "Covariance matrix must be square.");
    assert (cov.rows() == stateDim_ && "Covariance matrix size does not match.");

    Eigen::VectorXd vec(vecDim_);
    int k = 0;
    // loop to pick upper triangle
    for (int i = 0; i < stateDim_; ++i)
        for (int j = i; j < stateDim_; ++j)
            vec[k++] = cov(i, j);

    return vec;
}

Eigen::MatrixXd ompl::base::CovarianceStateSpace::vec2cov(const Eigen::VectorXd& vec) const
{
    assert (vec.size() == vecDim_ && "Vector size does not match state dimension.");

    Eigen::MatrixXd cov(stateDim_, stateDim_);
    int k = 0;
    for (int i = 0; i < stateDim_; ++i)
        for (int j = i; j < stateDim_; ++j)
        {
            cov(i, j) = vec[k];
            cov(j, i) = vec[k];
            ++k;
        }
    return cov;
}

void ompl::base::CovarianceStateSpace::registerProjections()
{
}