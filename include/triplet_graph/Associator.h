#ifndef TRIPLET_GRAPH_ASSOCIATOR_H_
#define TRIPLET_GRAPH_ASSOCIATOR_H_

#include <tue/config/configuration.h>

#include "triplet_graph/graph_types.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Path.h"
#include "triplet_graph/Graph.h"
#include "triplet_graph/ProbabilityCalculator.h"

#include <boost/shared_ptr.hpp>

namespace triplet_graph
{

class Associator
{
public:
    Associator();

    bool configure(tue::Configuration config);

    void setGraph(const Graph& graph);

    bool getAssociations(const Measurement &measurement, AssociatedMeasurement& associations, const int goal_node_i );

    bool getUnassociatedPoints( Measurement& unassociated_points );

    bool getPath(Path& path);

private:
    Measurement measurement_;
    Measurement unassociated_points_;
    const Graph* graph_ptr_;
    Path path_;

    std::vector<boost::shared_ptr<ProbabilityCalculator> > probCalculators_;
    std::vector<double> min_assoc_probs_;

    bool associated_;

    double best_association_prob_;

    double associate(const AssociatedMeasurement &graph_positions,
                     const Measurement &measurement,
                     AssociatedMeasurement& resulting_associations,
                     const ProbabilityCalculator& cost_calculator,
                     const double max_no_std_devs,
                     double parents_cost,
                     int level);

};

}

#endif
