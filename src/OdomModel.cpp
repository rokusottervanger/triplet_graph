#include "triplet_graph/OdomModel.h"

#ifndef PI
#define PI 3.14159265
#endif

namespace triplet_graph
{

double OdomModel::prob(double a, double b)
{
    /* Algorithm for computing the probability of 'a' under a zero-
         * centered normal distribution with standard deviation 'b'.
         */
    double b_sq = b*b;
    return exp(-0.5*a*a/b_sq) / sqrt(2*PI*b_sq);
}

double OdomModel::setErrorParams(double alpha1, double alpha2, double alpha3, double alpha4)
{
    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
}

double OdomModel::setOdomDelta( geo::Transform3d& odom_delta )
{
    odom_delta_ = odom_delta;
}

double OdomModel::getMotionProbability( geo::Transform3d& proposed_delta )
{
    double dx_bar = odom_delta_.t.getX();
    double dy_bar = odom_delta_.t.getY();
    double dth_bar = odom_delta_.getYaw();

    double dx = proposed_delta.t.getX();
    double dy = proposed_delta.t.getY();
    double dth = proposed_delta.getYaw();

    double delta_rot_1 = atan2(dy_bar, dx_bar);
    double delta_trans = sqrt(dx_bar*dx_bar + dy_bar*dy_bar);
    double delta_rot_2 = dth_bar - delta_rot_1;

    double delta_rot_1_hat = atan2(dy, dx);
    double delta_trans_hat = sqrt(dx*dx + dy*dy);
    double delta_rot_2_hat = dth - delta_rot_1_hat;

    double p1 = prob( delta_rot_1 - delta_rot_1_hat, alpha1_ * fabs(delta_rot_1_hat) + alpha2_ * delta_trans_hat );
    double p2 = prob( delta_trans - delta_trans_hat, alpha3_ * delta_trans_hat + alpha4_ * ( fabs(delta_rot_1_hat) + fabs(delta_rot_2_hat) ) );
    double p3 = prob( delta_rot_2 - delta_rot_2_hat, alpha1_ * fabs(delta_rot_2_hat) + alpha2_* delta_trans_hat );

    return p1 * p2 * p3;
}

} // End namespace triplet_graph
