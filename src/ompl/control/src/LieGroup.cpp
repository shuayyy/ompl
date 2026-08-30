/**
 * Authors: Anonymous
 * Date: 2025-07-07
 */

#include <cmath>
#include "ompl/control/LieGroup.h"

namespace oc = ompl::control;

std::vector<double> oc::logSE2(Eigen::Matrix3d &transform)
{   
    /*
    Logarithm map from SE(2) -> se(2).
    'transform' is a 3x3 homogeneous transformation matrix.
    Returns a 3x1 vector [deltaX, deltaY, omage] in tangent space.

    omega = arctan2(rot[1, 0], rot[0, 0])
    [dx, dy] = V(w)^-1 * t
    */
    Eigen::Matrix2d rot = transform.block<2, 2>(0, 0);
    Eigen::Vector2d t = transform.block<2, 1>(0, 2);

    // Extract rotation angle directly
    // The trace of R = 2 cos(omega), so omega = atan2(R[1,0], R[0,0])
    double omega = std::atan2(rot(1, 0), rot(0, 0));

    // If rotation is very small, approximate to avoid numerical issues
    Eigen::Vector2d deltaXY;
    if (std::abs(omega) < 1e-3)
    {
        // Both R and V are close to identity, so V^-1[deltaX, deltaY] = t
        deltaXY = t;
    }
    else
    {
        // For SE(2), we can use the closed-form formula for V(w)^-1
        // V_inv = omega / (2 * (1 - cos(w))) *
        //         [[    sin(w)   , 1 - cos(w)]
        //          [-(1 - cos(w)),   sin(w)  ]]
        double c = std::cos(omega), s = std::sin(omega);
        Eigen::Matrix2d V_inv = (omega / (2.0 * (1.0 - c))) * 
                                (Eigen::Matrix2d() << s, 1.0 - c,
                                                      c - 1.0, s).finished();
        deltaXY = V_inv * t;
    }
    
    return {deltaXY(0), deltaXY(1), omega};
}

Eigen::Matrix3d oc::expSE2(const std::vector<double> &xi)
{   
    /*
    Exponential map from se(2) -> SE(2).
    'xi' is a 3x1 vector [deltaX, deltaY, omage] in tangent space.
    Returns a 3x3 homogeneous transformation matrix.

    exp([v; w]) = [ R(w) , J(w)*v ]
                  [  0   ,    1   ]
    */
    double deltaX = xi[0], deltaY = xi[1], omega = xi[2];
    Eigen::Matrix2d rot;
    Eigen::Vector2d t;

    // If rotation is very small, use approximation
    if (std::abs(omega) < 1e-3)
    {
        rot = Eigen::Matrix2d::Identity();
        t = Eigen::Vector2d(deltaX, deltaY);
    }
    else
    {
        double c = std::cos(omega), s = std::sin(omega);
        rot = (Eigen::Matrix2d() << c, -s,
                                    s,  c).finished();
        // R(omega) = (sin(omega) / omega) * I + (1 - cos(omega)) / omega^2 * [omega]
        //          = (sin(omega) / omega) * I
        //          + (1 - cos(omega)) / omega^2 * [0, -omega; omega, 0]
        Eigen::Matrix2d rotOmega = (Eigen::Matrix2d() << s / omega, (c - 1) / omega,
                                                         (1 - c) / omega, s / omega).finished();
        t = rotOmega * (Eigen::Vector2d() << deltaX, deltaY).finished();
    }

    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    transform.block<2, 2>(0, 0) = rot;
    transform.block<2, 1>(0, 2) = t;
    return transform;
}

Eigen::Matrix3d oc::adjointSE2(Eigen::Matrix3d &transform)
{   /*
    Returns the 3x3 adjoint matrix Ad_T for T in SE(2).
    If T = [R   t]
           [0   1]
    then Ad_T = [[ R , [t]R ],
                 [ 0 ,  1   ]]
    */
    Eigen::Matrix2d rot = transform.block<2, 2>(0, 0);
    Eigen::Vector2d t = transform.block<2, 1>(0, 2);

    // [t]R in SE2 is simply -[1]t = [t1; -t0]
    Eigen::Vector2d tCrossRot = (Eigen::Vector2d() << t(1), -t(0)).finished();
    Eigen::Matrix3d adjoint = Eigen::Matrix3d::Zero();
    adjoint.block<2, 2>(0, 0) = rot;
    adjoint.block<2, 1>(0, 2) = tCrossRot;
    adjoint(2, 2) = 1.0;
    return adjoint;
}

Eigen::Matrix3d oc::rightJacobianSE2(const std::vector<double> &xi)
{   /*
    Returns the 3x3 right Jacobian matrix J for T in SE(2).
    J(xi) = [[ J(omega) , A(xi) ],
             [  0       ,  1    ]]
    */
    double deltaX = xi[0], deltaY = xi[1], omega = xi[2];
    Eigen::Matrix2d jacOmega;
    Eigen::Vector2d AXi;

    // If rotation is close to zero, use approximation
    if (std::abs(omega) < 1e-3)
    {
        // Use first order Taylor expansion for small angles
        jacOmega = Eigen::Matrix2d::Identity();
        AXi = (Eigen::Vector2d() << -deltaY / 2.0, -deltaX / 2.0).finished();
    }
    else
    {
        double c = std::cos(omega), s = std::sin(omega);
        double omegaSq = omega * omega;

        // J(omega) = [[sin(omega) / omega, (1 - cos(omega)) / omega],
        //             [(cos(omega) - 1) / omega, sin(omega) / omega]]
        jacOmega = (Eigen::Matrix2d() << s / omega, (1 - c) / omega,
                                         (c - 1) / omega, s / omega).finished();

        // A(xi) = 1 / omega^2 * [
        //                       dx(omega - sin(omega)) + dy(cos(omega) - 1),
        //                       dy(omega - sin(omega)) - dx(1 - cos(omega))
        //                       ]
        AXi = (Eigen::Vector2d() << (deltaX * (omega - s) + deltaY * (c - 1)),
                                    (deltaY * (omega - s) + deltaX * (1 - c))).finished() / omegaSq;
    }

    Eigen::Matrix3d jac = Eigen::Matrix3d::Identity();
    jac.block<2, 2>(0, 0) = jacOmega;
    jac.block<2, 1>(0, 2) = AXi;
    return jac;
}

Eigen::Matrix3d oc::invTransformSE2(Eigen::Matrix3d &transform)
{   /* Inverse of the SE(2) transform. */
    Eigen::Matrix2d rot = transform.block<2, 2>(0, 0);
    Eigen::Vector2d t = transform.block<2, 1>(0, 2);

    Eigen::Matrix3d invTransform = Eigen::Matrix3d::Identity();
    invTransform.block<2, 2>(0, 0) = rot.transpose();
    invTransform.block<2, 1>(0, 2) = -rot.transpose() * t;
    return invTransform;
}

std::vector<double> oc::getSE2Vec(Eigen::Matrix3d &transform)
{   /* Extract the (x, y, theta) from the SE(2) state. */
    double x = transform(0, 2), y = transform(1, 2);
    double theta = std::atan2(transform(1, 0), transform(0, 0));

    return {x, y, theta};
}

Eigen::Matrix3d oc::getTransform(const std::vector<double> &params)
{   /* Get the 3*3 homogeneous transformation matrix from the params (x, y, theta). */
    double x = params[0], y = params[1], theta = params[2];
    Eigen::Matrix3d transform = (Eigen::Matrix3d() << std::cos(theta), -std::sin(theta), x,
                                                      std::sin(theta),  std::cos(theta), y,
                                                      0, 0, 1).finished();
    return transform;
}