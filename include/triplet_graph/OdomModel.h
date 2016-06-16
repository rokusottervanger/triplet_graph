#ifndef TRIPLET_GRAPH_ODOM_MODEL_H_
#define TRIPLET_GRAPH_ODOM_MODEL_H_

#include <geolib/math_types.h>

namespace triplet_graph
{

class OdomModel
{
    double alpha1_, alpha2_, alpha3_, alpha4_;

    geo::Transform3d odom_delta_;

    static double prob(double a, double b);

public:
    double setErrorParams(double alpha1, double alpha2, double alpha3, double alpha4);

    double setOdomDelta( geo::Transform3d& odom_delta );

    double getMotionProbability( geo::Transform3d& proposed_delta );
};

}

#endif
