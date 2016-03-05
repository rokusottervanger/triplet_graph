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

bool Associator::configure()
{
    max_association_dist_ = 0.05;
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

double Associator::associate(const Measurement& graph_positions, const Measurement& measurement, AssociatedMeasurement& resulting_associations)
{
    // Base case
    // ------------------------------

    if ( measurement.points.size() == 0 )
    {
        return 0.0;
    }

    // Recursive case
    // ------------------------------

    //    std::vector<geo::Vec3d> prediction;
    //    nearestNeighbor(measurement, prediction);

    // Take a point from the measurement to associate with graph nodes
    typedef std::pair<geo::Vec3d,double> Point;
    Point point(measurement.points.back(),measurement.uncertainties.back());

    Measurement reduced_measurement = measurement;
    reduced_measurement.pop_back();

    double best_dist = 1e9;
    double total_dist;

    for ( int i = 0; i < graph_positions.points.size(); i++ )
    {
        AssociatedMeasurement associations;
        Measurement reduced_graph_positions;

        reduced_graph_positions = graph_positions;
        reduced_graph_positions.points.erase(reduced_graph_positions.points.begin()+i);
        reduced_graph_positions.uncertainties.erase(reduced_graph_positions.uncertainties.begin()+i);

        double local_dist = (point.first - graph_positions.points[i]).length2(); // TODO: This is only the squared euclidian distance, go for something like mahalanobis.
        //        if ( local_dist < best_dist ) // TODO: uncomment this and check performance improvement
        //        {
        double dist = local_dist + associate(reduced_graph_positions, reduced_measurement, associations); // TODO: reduce size of graph positions for recursion to work!

        if ( dist < best_dist )
        {
            best_dist = dist;
            resulting_associations = associations;
        }
        //        }
    }

    return total_dist;

    //  - Hypothesize association
    //  - If possible, make prediction about nodes rigidly connected (object)
    //  - Do nearest neighbor association on this object
    //  - Decide if correct association or not.
    //      - If confirmed, add hypothesis and resulting nearest neighbor associations to associations and proceed
    //      - If not confirmed, go back, try another hypothesis until out of (reasonable) combinations and proceed

    // If this takes too long,
    return false;
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

bool Associator::getAssociations( const Measurement& measurement, AssociatedMeasurement& associations )
{
    // TODO: this is all kind of hacky. Can some stuff be recycled, cached should positions not be calculated in the association algorithm?
    // TODO: Goal node other than -1
    std::cout << "Going to try to associate!" << std::endl;
    std::cout << "Putting old associations in positions vector" << std::endl;
    std::vector<geo::Vec3d> positions(graph_ptr_->size());
    for ( int i = 0; i < associations_.nodes.size(); i++ )
    {
        positions[associations_.nodes[i]] = associations_.measurement.points[i];
    }
    std::cout << "Calculating path..." << std::endl;
    PathFinder pathFinder(*graph_ptr_, associations_.nodes);
    Path path;
    pathFinder.findPath(-1,path);

    std::cout << "Calculating positions... " << std::endl;
    calculatePositions(*graph_ptr_, positions, path);
    Measurement graph_positions;
    graph_positions.points = positions;

    std::cout << "Associating... " << std::endl;
    associate(graph_positions,measurement,associations);

    std::cout << "Done!" << std::endl;

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
