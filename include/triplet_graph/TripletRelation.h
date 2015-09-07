#ifndef GRAPH_MAP_TRIPLET_RELATION_H_
#define GRAPH_MAP_TRIPLET_RELATION_H_

#include <geolib/datatypes.h>

namespace triplet_graph_map
{

struct TripleRelation
{
    double side1_, side2_, side3_;
    // TODO: add some form of uncertainty
};

}

#endif
