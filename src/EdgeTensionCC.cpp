#include "triplet_graph/EdgeTensionCC.h"

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Path.h"

#include <geolib/math_types.h>

namespace triplet_graph
{

double EdgeTensionCC::calculateCost(const Graph& graph,
                                    const geo::Vec3d& cur_measurement_pt,
                                    const double cur_measurement_std_dev,
                                    const double odom_std_dev,
                                    const AssociatedMeasurement& graph_positions,
                                    const int node_index,
                                    const AssociatedMeasurement& input_associations,
                                    Path& path) const
{
    double cur_measurement_std_dev_sq = cur_measurement_std_dev * cur_measurement_std_dev;
    double odom_std_dev_sq = odom_std_dev * odom_std_dev;

    // Get parent nodes from path (graph_positions is constructed in the order of the path, but nodes are removed in recursion)
    int node_i = graph_positions.nodes[node_index]; // node index in graph
    int path_index = path.node_indices[node_i];
    int parent_1_i = path.parent_tree[path_index].first;
    int parent_2_i = path.parent_tree[path_index].second;

    // If root node, calculation goes differently
    if ( parent_1_i == -1 || parent_2_i == -1 )
    {
        // Use the difference vector between the current point and the predicted position of the current node,
        // and calculate the local cost of associating the node with the current point using the same edge
        // stretch method as in the non-root node case, only without the edge error (but later including an
        // odometry error model).
        // TODO: take into account odom error when trying to associate root nodes
        return exp(-(cur_measurement_pt - graph_positions.measurement.points[node_index]).length2()/ ( cur_measurement_std_dev_sq * odom_std_dev_sq ));
    }
    else
    {
        // Get edge lengths of current graph node to its parents in the path
        Graph::const_iterator node_it = graph.begin()+graph_positions.nodes[node_index];

        Graph::const_edge2_iterator edge_1_it = graph.beginEdges() + node_it->edgeByPeer(parent_1_i);
        Graph::const_edge2_iterator edge_2_it = graph.beginEdges() + node_it->edgeByPeer(parent_2_i);

        Graph::const_edge3_iterator trip_it = graph.beginTriplets() + edge_1_it->tripletByNode(parent_2_i);

        // Get the most recent positions of the parent nodes (either the predicted position or the hypothesized associated measurement point),
        geo::Vec3d parent_1_pos = getMostRecentNodePosition(input_associations, graph_positions, parent_1_i);
        geo::Vec3d parent_2_pos = getMostRecentNodePosition(input_associations, graph_positions, parent_2_i);

        // calculate the vector between the current measurement point and the node's parents
        geo::Vec3d v_1_m = cur_measurement_pt - parent_1_pos;
        geo::Vec3d v_2_m = cur_measurement_pt - parent_2_pos;

        // Check if hypothesis satisfies triplet:
        // calculate the cross product of the vectors to the node's parents
        double sign = v_1_m.cross(v_2_m).z;

        // create a dummy triplet from the current point and its parents
        Edge3 t(graph_positions.nodes[node_index],parent_1_i,parent_2_i);

        if ( sign < 0 && t == *trip_it || sign > 0 && t.flip() == *trip_it )
            return 0.0; // todo: this is very strict. Flat triplets may sometimes invert. How to handle that?

        // and calculate the lengths of those vectors
        double l_1_m = v_1_m.length();
        double l_2_m = v_2_m.length();

        // Calculate the elongation of the edges
        double e1 = l_1_m - edge_1_it->l;
        double e2 = l_2_m - edge_2_it->l;

        // Calculate the 'stress' using the variance in the edge as well as the variance of the measurement. todo: Check the math on this
        double stddev1 = edge_1_it->std_dev * edge_1_it->l;
        double stddev2 = edge_2_it->std_dev * edge_2_it->l;

        double s1 = e1*e1 / ( stddev1*stddev1 + cur_measurement_std_dev_sq );
        double s2 = e2*e2 / ( stddev2*stddev2 + cur_measurement_std_dev_sq );

        // Calculate the direction vectors of the 'forces' working on the graph node to pull it to the measurement point
        geo::Vec3d dir_1 = v_1_m/l_1_m;
        geo::Vec3d dir_2 = v_2_m/l_2_m;

        // Calculate the resulting 'force' on the node
        return exp(-(s1*dir_1 + s2*dir_2).length());
    }
}

// -----------------------------------------------------------------------------------------------

geo::Vec3d EdgeTensionCC::getMostRecentNodePosition(const AssociatedMeasurement& associations, const AssociatedMeasurement& graph_positions, const int node_i) const
{
    std::map<int,int>::const_iterator index_it = associations.node_indices.find(node_i);

    if ( index_it == associations.node_indices.end() )
    {
        index_it = graph_positions.node_indices.find(node_i);
        const int index = index_it->second;
        return graph_positions.measurement.points[ index ];
    }
    else
    {
        const int index = index_it->second;
        return associations.measurement.points[ index ];
    }

}

}
