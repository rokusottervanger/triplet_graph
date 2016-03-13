#include "triplet_graph/Associator.h"

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/graph_operations.h"
#include "triplet_graph/PathFinder.h"

namespace triplet_graph
{

Associator::Associator():
    associated_(false)
{}

bool Associator::configure(tue::Configuration config)
{
    if ( config.readGroup("association") )
    {
        config.value("max_association_distance", max_association_dist_ );
        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "[ASSOCIATOR] Configure: No configuration for association found!" << "\033[0m" << std::endl;
        return false;
    }
    max_association_dist_sq_ = max_association_dist_*max_association_dist_;
    return true;
}

// -----------------------------------------------------------------------------------------------

void Associator::setAssociations(const AssociatedMeasurement& associations)
{
    associations_ = associations;
    associated_ = false;
}

// -----------------------------------------------------------------------------------------------

void Associator::setGraph(const Graph& graph)
{
    graph_ptr_ = &graph;
}

// -----------------------------------------------------------------------------------------------

double Associator::associate(const AssociatedMeasurement& graph_positions, const Measurement& measurement, AssociatedMeasurement& resulting_associations)
{

    // ------------------------------
    // ------------------------------
    // BASE CASE
    // ------------------------------
    // ------------------------------

    if ( measurement.points.size() == 0 )
    {
        resulting_associations.measurement.frame_id = measurement.frame_id;
        resulting_associations.measurement.time_stamp = measurement.time_stamp;
        return 0.0;
    }


    // ------------------------------
    // ------------------------------
    // RECURSIVE CASE
    // ------------------------------
    // ------------------------------

    // Take a measurement point from the measurement to associate
    geo::Vec3d cur_measurement_pt = measurement.points.back();
    Measurement reduced_measurement = measurement;
    reduced_measurement.points.pop_back();


    // Hypothesize that the measurement point does not associate at all
    // ------------------------------

    // Give this the highest possible cost, so that a better association is always preferred
    double local_cost = max_association_dist_sq_;

    // Copy all graph positions (because nothing was associated, all of them are passed to the next recursion)
    AssociatedMeasurement reduced_graph_positions;
    reduced_graph_positions = graph_positions;
    AssociatedMeasurement associations;

    // Calculate further associations and set this association and its cost as the benchmark for other associations
    double best_cost = local_cost + associate( reduced_graph_positions, reduced_measurement, associations );
    resulting_associations = associations;


    // Hypothesize association with every graph node
    // ------------------------------

    int best_node = -1;

    for ( int i = 0; i < graph_positions.measurement.points.size(); i++ )
    {
        // Calculate cost of currently hypothesized association
        local_cost = (cur_measurement_pt - graph_positions.measurement.points[i]).length2(); // TODO: This is only the squared euclidian distance, go for something like mahalanobis.

        // If local hypothesis cost is low enough...
        if ( local_cost < max_association_dist_sq_ )
        {
            // create a new measurement for the graph positions reduced by the locally hypothesized node
            reduced_graph_positions = graph_positions;
            reduced_graph_positions.measurement.points.erase(reduced_graph_positions.measurement.points.begin()+i);
            reduced_graph_positions.nodes.erase(reduced_graph_positions.nodes.begin()+i);

            // Calculate further associations given current hypothesis
            AssociatedMeasurement associations;
            double cost = local_cost + associate(reduced_graph_positions, reduced_measurement, associations);

            // Remember the lowest association cost, its resulting associations and the corresponding hypothesis
            if ( cost < best_cost )
            {
                best_cost = cost;
                resulting_associations = associations;
                best_node = graph_positions.nodes[i];
            }
        }
    }


    // Check what was the best solution
    // ------------------------------

    // If best node was set, there was a good association, so store that in the resulting associations
    if ( best_node > -1 )
    {
        resulting_associations.measurement.points.push_back(cur_measurement_pt);
        resulting_associations.nodes.push_back(best_node);
    }

    return best_cost;
}

// -----------------------------------------------------------------------------------------------

double Associator::associateFancy(const AssociatedMeasurement& graph_positions, const Measurement& measurement, AssociatedMeasurement& resulting_associations)
{

    // ------------------------------
    // ------------------------------
    // BASE CASE
    // ------------------------------
    // ------------------------------

    if ( measurement.points.size() == 0 )
    {
        resulting_associations.measurement.frame_id = measurement.frame_id;
        resulting_associations.measurement.time_stamp = measurement.time_stamp;
        return 0.0;
    }


    // ------------------------------
    // ------------------------------
    // RECURSIVE CASE
    // ------------------------------
    // ------------------------------

    // Take a measurement point from the measurement to associate
    geo::Vec3d cur_measurement_pt = measurement.points.back();
    Measurement reduced_measurement = measurement;
    reduced_measurement.points.pop_back();


    // Hypothesize that the measurement point does not associate at all
    // ------------------------------

    // NOT SURE IF THIS PART IS STILL VALID...

    // Give this the highest possible cost, so that a better association is always preferred
    double local_cost = max_association_dist_sq_;

    // Copy all graph positions (because nothing was associated, all of them are passed to the next recursion)
    AssociatedMeasurement reduced_graph_positions;
    reduced_graph_positions = graph_positions;
    AssociatedMeasurement associations;

    // Calculate further associations and set this association and its cost as the benchmark for other associations
    double best_cost = local_cost + associateFancy( reduced_graph_positions, reduced_measurement, associations );
    resulting_associations = associations;

    // ... UP TO HERE.


    // Hypothesize association with every graph node
    // ------------------------------

    // THIS IS GOING TO TAKE MUCH LONGER BECAUSE I'M NOT REJECTING ANY HYPOTHESIS (FOR EXAMPLE BASED ON A MAX ASSOCIATION DISTANCE)

    int best_node = -1;

    for ( int i = 0; i < graph_positions.measurement.points.size(); i++ )
    {
        // create a new measurement for the graph positions reduced by the locally hypothesized node
        reduced_graph_positions = graph_positions;
        reduced_graph_positions.measurement.points.erase(reduced_graph_positions.measurement.points.begin()+i);
        reduced_graph_positions.nodes.erase(reduced_graph_positions.nodes.begin()+i);

        // Calculate further associations given current hypothesis
        AssociatedMeasurement associations;
        double further_cost = associateFancy(reduced_graph_positions, reduced_measurement, associations);

        // Calculate local cost using most recend parent positions (use position from associations if possible, otherwise use calculated position from graph_positions)
        double local_cost = 0;

        double total_cost = further_cost + local_cost;

        // Remember the lowest association cost, its resulting associations and the corresponding hypothesis
        if ( total_cost < best_cost )
        {
            best_cost = total_cost;
            resulting_associations = associations;
            best_node = graph_positions.nodes[i];
        }
    }


    // Check what was the best solution
    // ------------------------------

    // If best node was set, there was a good association, so store that in the resulting associations
    if ( best_node > -1 )
    {
        resulting_associations.measurement.points.push_back(cur_measurement_pt);
        resulting_associations.nodes.push_back(best_node);
    }

    return best_cost;
}

// -----------------------------------------------------------------------------------------------

void Associator::nearestNeighbor( const Measurement& measurement, const std::vector<geo::Vec3d> prediction )
{
    for ( std::vector<geo::Vec3d>::const_iterator it_m = measurement.points.begin(); it_m != measurement.points.end(); ++it_m )
    {
        double best_dist_sq = 1.0e9;
        int best_guess = -1;
        geo::Vec3d best_pos;
        double max_dist_sq = max_association_dist_ * max_association_dist_;

        // Go through nodes in path (nodes to be associated from far to close) to check if one associates with the measured point
        for ( Path::iterator it_p = path_.begin(); it_p != path_.end(); ++it_p )
        {
            int i = *it_p;
            double dx_sq = (*it_m - prediction[i]).length2();
            if ( dx_sq < max_dist_sq )
            {
                if ( dx_sq < best_dist_sq )
                {
                    best_dist_sq = dx_sq;
                    best_pos = *it_m;
                    best_guess = i;
                }
            }
        }

        // Check if an association is made, and if so, push it into associations
        if ( best_guess > -1 )
        {
            associations_.nodes.push_back(best_guess);
            associations_.measurement.points.push_back(best_pos);
        }
        else
        {
            unassociated_points_.points.push_back(*it_m);
        }
    }
}

// -----------------------------------------------------------------------------------------------

Graph Associator::getObjectSubgraph( const Graph& graph, const int node_i )
{
    // THIS FUNCTION MAY RESULT IN A GRAPH WITHOUT EDGE3S!!!
    Graph object;

    std::ostringstream node_id;
    node_id << node_i;
    object.addNode(node_id.str());

    std::vector<Node> nodes = graph.getNodes();
    Node base_node = nodes[node_i];
    std::vector<int> peers = base_node.getPeers();

    std::vector<Edge2> edges = graph.getEdge2s();
    std::vector<Edge3> trips = graph.getEdge3s();

    for ( std::vector<int>::const_iterator it = peers.begin(); it != peers.end(); ++it )
    {
        int e = base_node.edgeByPeer(*it);
        if ( edges[e].rigid )
        {
            std::ostringstream peer_id;
            peer_id << *it;
            object.addNode(peer_id.str());
            object.addEdge2(node_i,*it,edges[e].l);
        }
    }

    return object;
}

// -----------------------------------------------------------------------------------------------

bool Associator::getAssociations( const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
{
    // Find a path through the graph starting from the associated nodes
    PathFinder pathFinder( *graph_ptr_, associations_.nodes );
    pathFinder.findPath( goal_node_i, path_ );

    // Put the known positions (from given associations) in the positions vector
    std::vector<geo::Vec3d> positions( graph_ptr_->size() );
    for ( int i = 0; i < associations_.nodes.size(); i++ )
    {
        positions[ associations_.nodes[i] ] = associations_.measurement.points[i];
    }

    // Calculate the positions of graph nodes on the path
    calculatePositions( *graph_ptr_, positions, path_ );
    AssociatedMeasurement graph_positions;
    graph_positions.measurement.frame_id   = measurement.frame_id;
    graph_positions.measurement.time_stamp = measurement.time_stamp;
    graph_positions.measurement.points     = positions;

    for ( int i = 0; i < graph_positions.measurement.points.size(); i++ )
    {
        graph_positions.nodes.push_back(i);
    }

    // Call the recursive association algorithm
    associate( graph_positions, measurement, associations );

    associated_ = true;

    return true;
}

// -----------------------------------------------------------------------------------------------

bool Associator::getUnassociatedPoints( Measurement& unassociated_points )
{
    if ( associated_ )
    {
        unassociated_points = unassociated_points_;
        return true;
    }
    else
        return false;
}

// -----------------------------------------------------------------------------------------------

bool Associator::getPath(Path& path)
{
    if ( associated_ )
    {
        path = path_;
        return true;
    }
    else
        return false;
}

}
