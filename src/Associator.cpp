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
        config.value("max_no_std_devs", max_no_std_devs_ );
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
    associated_ = false;
}

// -----------------------------------------------------------------------------------------------

double Associator::associate(const AssociatedMeasurement& graph_positions, const Measurement& measurement, AssociatedMeasurement& resulting_associations)
{
    calls_++;

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
    reduced_measurement.uncertainties.pop_back();



    // Hypothesize that the measurement point does not associate at all
    // ------------------------------

    // This sets the threshold for association
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
            reduced_graph_positions.measurement.uncertainties.erase(reduced_graph_positions.measurement.uncertainties.begin()+i);
            reduced_graph_positions.node_indices.erase(reduced_graph_positions.nodes[i]);
            // TODO: hack, fix more elegantly (?)
            for ( std::map<int,int>::iterator it = reduced_graph_positions.node_indices.begin(); it != reduced_graph_positions.node_indices.end(); ++it )
            {
                if ( it->second > i )
                {
                    it->second -= 1;
                }
            }
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
        resulting_associations.node_indices[best_node] = resulting_associations.nodes.size();
        resulting_associations.nodes.push_back(best_node);
    }

    return best_cost;
}

// -----------------------------------------------------------------------------------------------

double Associator::associateFancy( const AssociatedMeasurement& graph_positions, const Measurement& measurement, AssociatedMeasurement& associations)
{
    AssociatedMeasurement input_associations = associations;

    calls_++;


    // ------------------------------
    // ------------------------------
    // BASE CASE
    // ------------------------------
    // ------------------------------

    if ( measurement.points.size() == 0 )
    {
        associations.measurement.frame_id = measurement.frame_id;
        associations.measurement.time_stamp = measurement.time_stamp;
        return 0.0;
    }



    // ------------------------------
    // ------------------------------
    // RECURSIVE CASE
    // ------------------------------
    // ------------------------------

    // Take a measurement point from the measurement to associate
    geo::Vec3d cur_measurement_pt = measurement.points.back();
    double cur_measurement_std_dev = measurement.uncertainties.back();
    double cur_measurement_std_dev_sq = cur_measurement_std_dev*cur_measurement_std_dev;
    Measurement reduced_measurement = measurement;
    reduced_measurement.points.pop_back();
    reduced_measurement.uncertainties.pop_back();


    // Hypothesize that the measurement point does not associate at all
    // ------------------------------

    // This sets the threshold for association
    double local_cost = max_no_std_devs_;

    // Copy all graph positions (because nothing was associated, all of them are passed to the next recursion)
    AssociatedMeasurement reduced_graph_positions = graph_positions;
    AssociatedMeasurement hyp_associations = input_associations;

    // Calculate further associations and set this association and its cost as the benchmark for other associations
    double best_cost = local_cost + associateFancy( reduced_graph_positions, reduced_measurement, hyp_associations );


    // Hypothesize association with every graph node
    // ------------------------------

    int best_node = -1;

    for ( int i = 0; i < graph_positions.measurement.points.size(); i++ )
    {

        // Calculate cost of hypothesized association
        // ------------------------------
        // Calculate local cost using most recent parent positions (use position from associations if possible, otherwise use calculated position from graph_positions)

        // Get parent nodes from path (graph_positions is constructed in the order of the path, but nodes are removed in recursion)
        int node_i = graph_positions.nodes[i]; // node index in graph
        int path_index = path_.node_indices[node_i];
        int parent_1_i = path_.parent_tree[path_index].first;
        int parent_2_i = path_.parent_tree[path_index].second;

        // If root node, calculation goes differently
        if ( parent_1_i == -1 || parent_2_i == -1 )
        {
            // Use the difference vector between the current point and the predicted position of the current node,
            // and calculate the local cost of associating the node with the current point using the same edge
            // stretch method as in the non-root node case, only without the edge error (but later including an
            // odometry error model), the edge being the distance from the sensor.
            // TODO: take into account odom error when trying to associate root nodes
            local_cost = (cur_measurement_pt - graph_positions.measurement.points[i]).length2()/cur_measurement_std_dev_sq;
//            local_cost = (cur_measurement_pt - graph_positions.measurement.points[i]).length2()/(cur_measurement_std_dev + odom_covariance * cur_measurement_pt.normalized());
        }
        else
        {
            // Get edge lengths of current graph node to its parents in the path
            Graph::const_iterator node_it = graph_ptr_->begin()+graph_positions.nodes[i];

            Graph::const_edge2_iterator edge_1_it = graph_ptr_->beginEdges() + node_it->edgeByPeer(parent_1_i);
            Graph::const_edge2_iterator edge_2_it = graph_ptr_->beginEdges() + node_it->edgeByPeer(parent_2_i);

            Graph::const_edge3_iterator trip_it = graph_ptr_->beginTriplets() + edge_1_it->tripletByNode(parent_2_i);

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
            Edge3 t(graph_positions.nodes[i],parent_1_i,parent_2_i);

            if ( sign < 0 && t == *trip_it || sign > 0 && t.flip() == *trip_it )
                continue; // TODO: Verify that this works!

            // and calculate the lengths of those vectors
            double l_1_m = v_1_m.length();
            double l_2_m = v_2_m.length();

            // Calculate the elongation of the edges
            double e1 = l_1_m - edge_1_it->l;
            double e2 = l_2_m - edge_2_it->l;

            // Calculate the 'stress' using the variance in the edge as well as the variance of the measurement TODO: Is this a mathematically correct way to do this???
            double stddev1 = edge_1_it->std_dev * edge_1_it->l;
            double stddev2 = edge_2_it->std_dev * edge_2_it->l;

            double s1 = e1*e1 / ( stddev1*stddev1 + cur_measurement_std_dev_sq );
            double s2 = e2*e2 / ( stddev2*stddev2 + cur_measurement_std_dev_sq );

            // Calculate the direction vectors of the 'forces' working on the graph node to pull it to the measurement point
            geo::Vec3d dir_1 = v_1_m/l_1_m;
            geo::Vec3d dir_2 = v_2_m/l_2_m;

            // Calculate the resulting 'force' on the node
            local_cost = (s1*dir_1 + s2*dir_2).length();
        }

        // Only if local cost is lower than the threshold for the total cost, proceed with further associations
        if ( local_cost < max_no_std_devs_ )
        {

            // Add association to progressing hypothesis (which is passed on to further recursions)
            AssociatedMeasurement prog_associations = input_associations;

            prog_associations.measurement.points.push_back(cur_measurement_pt);
            prog_associations.measurement.uncertainties.push_back(cur_measurement_std_dev);
            prog_associations.node_indices[node_i] = prog_associations.nodes.size();
            prog_associations.nodes.push_back(node_i);


            // create a new measurement for the graph positions reduced by the locally hypothesized node
            reduced_graph_positions = graph_positions;
            reduced_graph_positions.measurement.points.erase(reduced_graph_positions.measurement.points.begin()+i);
            reduced_graph_positions.measurement.uncertainties.erase(reduced_graph_positions.measurement.uncertainties.begin()+i);
            reduced_graph_positions.nodes.erase(reduced_graph_positions.nodes.begin()+i);
            for ( std::map<int,int>::iterator it = reduced_graph_positions.node_indices.begin(); it != reduced_graph_positions.node_indices.end(); ++it )
            {
                if ( it->second > i )
                {
                    it->second -= 1; // TODO: hack, fix more elegantly (?)
                }
            }


            // Calculate the total force needed for the currently assumed associations and resulting best associations
            double total_cost = local_cost + associateFancy( reduced_graph_positions, reduced_measurement, prog_associations );

            // Remember the lowest association cost, its resulting associations and the corresponding hypothesis
            if ( total_cost < best_cost )
            {
                best_cost = total_cost;
                hyp_associations = prog_associations;
                best_node = graph_positions.nodes[i];

            }
        }
    }

    associations = hyp_associations; // TODO: maybe this distinction is not necessary, check that, because it's an extra copy.

    return best_cost;
}


// -----------------------------------------------------------------------------------------------

geo::Vec3d Associator::getMostRecentNodePosition(const AssociatedMeasurement& associations, const AssociatedMeasurement& graph_positions, int node_i)
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

// -----------------------------------------------------------------------------------------------

bool Associator::getAssociations( const Graph& graph, const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
{
    calls_ = 0;

    measurement_ = measurement;

    // Find a path through the graph starting from the associated nodes
    PathFinder pathFinder( *graph_ptr_, associations_.nodes );
    pathFinder.findPath( goal_node_i, path_ );

    std::cout << "[ASSOCIATOR]: path_:\n" << path_ << std::endl;

    // Put the known positions (from given associations) in the positions vector
    std::vector<geo::Vec3d> positions( graph_ptr_->size() );
    for ( int i = 0; i < associations_.nodes.size(); i++ )
    {
        positions[ associations_.nodes[i] ] = associations_.measurement.points[i];
    }

    // Calculate the positions of graph nodes on the path
    calculatePositions( *graph_ptr_, positions, path_ );
    AssociatedMeasurement path_positions;
    path_positions.measurement.frame_id = measurement.frame_id;
    path_positions.measurement.time_stamp = measurement.time_stamp;

    std::cout << "[ASSOCIATOR]: Path positions: " << std::endl;
    for ( int i = 1; i <= path_.size(); ++i )
    {
        // Calculate index in path
        int index = path_.size()-i; // Assumes order in path!!!!!!!

        path_positions.node_indices[path_[index]] = path_positions.nodes.size();
        path_positions.nodes.push_back(path_[index]);
        path_positions.measurement.points.push_back(positions[path_[index]]);
        path_positions.measurement.uncertainties.push_back(path_.costs[index]);

        std::cout << "[ASSOCIATOR]: Node " << path_positions.nodes.back() << " has position " << path_positions.measurement.points.back() << std::endl;
    }

    // Call the recursive association algorithms
    associateFancy( path_positions, measurement, associations );

//    for ( std::vector<int>::const_iterator it = associations.nodes.begin(); it != associations.nodes.end(); ++it )
    for ( int i = 0; i < associations.nodes.size(); ++i )
    {
        std::cout << "[ASSOCIATOR]: Associating node " << associations.nodes[i] << " with point at " << associations.measurement.points[i] << std::endl;
    }
    std::cout << std::endl;

    associated_ = true;

    std::cout << "number of (recursive) function calls: " << calls_ << std::endl;

    return true;
}

// -----------------------------------------------------------------------------------------------

//bool Associator::getAssociations( const Graph& graph, const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
//{
//    calls_ = 0;

//    // Find a path through the graph starting from the associated nodes
//    PathFinder pathFinder( *graph_ptr_, associations_.nodes );
//    pathFinder.findPath( goal_node_i, path_ );

//    // Put the known positions (from given associations) in the positions vector
//    std::vector<geo::Vec3d> positions( graph_ptr_->size() );
//    for ( int i = 0; i < associations_.nodes.size(); i++ )
//    {
//        positions[ associations_.nodes[i] ] = associations_.measurement.points[i];
//    }

//    // Calculate the positions of graph nodes on the path
//    calculatePositions( *graph_ptr_, positions, path_ );
//    AssociatedMeasurement path_positions;
//    path_positions.measurement.frame_id = measurement.frame_id;
//    path_positions.measurement.time_stamp = measurement.time_stamp;

//    for ( int i = 1; i <= path_.size(); ++i )
//    {
//        // Calculate index in path
//        int index = path_.size()-i; // Assumes order in path!!!!!!!

//        path_positions.node_indices[path_[index]] = path_positions.nodes.size();
//        path_positions.nodes.push_back(path_[index]);
//        path_positions.measurement.points.push_back(positions[path_[index]]);
//        path_positions.measurement.uncertainties.push_back(path_.costs[index]);
//    }

//    // Call the recursive association algorithms
//    // Nearest neighbor association
//    associate( path_positions, measurement, associations );

//    Measurement reduced_measurement = measurement;

//    for ( int i = 0; i < associations.nodes.size(); ++i )
//    {
//        int node = associations.nodes[i];
//        int path_index = path_positions.node_indices[node];

//        path_positions.measurement.points.erase( path_positions.measurement.points.begin() + path_index );
//        path_positions.measurement.uncertainties.erase( path_positions.measurement.uncertainties.begin() + path_index );
//        path_positions.nodes.erase( path_positions.nodes.begin() + path_index );
//        path_positions.node_indices.erase(node);

//        geo::Vec3d point = associations.measurement.points[i];
//        std::vector<geo::Vec3d>::iterator m_it = std::find( reduced_measurement.points.begin(), reduced_measurement.points.end(), point);
//        reduced_measurement.points.erase(m_it);
//        reduced_measurement.uncertainties.erase(reduced_measurement.uncertainties.begin()+ (reduced_measurement.points.end() - m_it));

//    }

//    // Edge tension association
//    associateFancy( path_positions, reduced_measurement, associations );

//    associated_ = true;

//    std::cout << "number of (recursive) function calls: " << calls_ << std::endl;

//    return true;
//}

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
