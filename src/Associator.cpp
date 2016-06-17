#include "triplet_graph/Associator.h"

#include <queue>

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/graph_operations.h"
#include "triplet_graph/PathFinder.h"
#include "triplet_graph/EdgeTensionCC.h"
#include "triplet_graph/NearestNeighborCC.h"

namespace triplet_graph
{

Associator::Associator():
    associated_(false)
{}

bool Associator::configure(tue::Configuration config)
{
    if ( config.readGroup("association") )
    {
        if ( config.readArray("association_modules",tue::REQUIRED) )
        {
            while ( config.nextArrayItem() )
            {
                std::string module_type;
                config.value("type", module_type );
                if ( module_type == "nearest_neighbor" )
                {
                    boost::shared_ptr<NearestNeighborCC> costCalculator(new NearestNeighborCC);
                    costCalculators_.push_back(costCalculator);

                    double max_no_std_devs;
                    if ( !config.value("max_no_std_devs", max_no_std_devs) )
                        std::cout << "[ASSOCIATOR] Configure: Warning! No max_no_std_devs defined for " << module_type << " cost calculator module" << std::endl;

                    max_assoc_dists_.push_back(max_no_std_devs);
                }
                else if ( module_type == "edge_tension" )
                {
                    boost::shared_ptr<EdgeTensionCC> costCalculator(new EdgeTensionCC);
                    costCalculators_.push_back(costCalculator);

                    double max_no_std_devs;
                    if ( !config.value("max_no_std_devs", max_no_std_devs) )
                        std::cout << "[ASSOCIATOR] Configure: Warning! No max_no_std_devs defined for " << module_type << " cost calculator module" << std::endl;

                    max_assoc_dists_.push_back(max_no_std_devs);
                }
                else
                {
                    std::cout << "[ASSOCIATOR] Configure: Warning! Unknown association cost module type given in config" << std::endl;
                }
            }
            config.endArray();

            if ( !costCalculators_.size() )
            {
                std::cout << "\033[31m" << "[ASSOCIATOR] Configure: No cost calculator modules available!" << "\033[0m" << std::endl;
                return false;
            }

        }
        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "[ASSOCIATOR] Configure: No configuration for association found!" << "\033[0m" << std::endl;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------------------------

void Associator::setGraph(const Graph& graph)
{
    graph_ptr_ = &graph;
    associated_ = false;
}

// -----------------------------------------------------------------------------------------------

// TODO's:
// - Check for a point surplus when not associating and penalize appropriately


double Associator::associate(const AssociatedMeasurement& graph_positions,
                             const Measurement& measurement,
                             AssociatedMeasurement& associations,
                             const CostCalculator& cost_calculator,
                             const double max_no_std_devs,
                             const double parents_cost=0.0,
                             int level=0)
{
//    std::stringstream indent;
//    while ( indent.str().size() < level*2 )
//        indent << " ";

    // BASE CASE
    // ------------------------------

    if ( level == graph_positions.nodes.size() )
    {
        double return_cost = measurement.points.size() * max_no_std_devs;
        double total_cost = return_cost + parents_cost;

        if ( total_cost < best_association_cost_ )
        {
            best_association_cost_ = total_cost;
        }

//        std::cout << indent.str() << "Returning " << return_cost << ", because " << measurement.points.size() << " points left" << std::endl;
        return return_cost;
    }
    if ( measurement.points.size() == 0 )
    {
//        std::cout << indent.str() << "Returning 0, because measurement is empty" << std::endl;
        if ( parents_cost < best_association_cost_ )
        {
            best_association_cost_ = parents_cost;
        }
        return 0.0;
    }


    // RECURSIVE CASE
    // ------------------------------

    int current_node = graph_positions.nodes[level];

    // If current node is already in the associated nodes, directly call next recursion for next node
    if ( associations.node_indices.find(current_node) != associations.node_indices.end() )
    {
        return associate( graph_positions, measurement, associations, cost_calculator, max_no_std_devs, parents_cost, level+1 );
    }

    std::priority_queue<std::pair<double,int>, std::vector< std::pair<double, int> >, std::greater<std::pair<double, int> > > Q;

    // Hypothesize no association
    Q.push(std::make_pair(max_no_std_devs,-1));

    // Hypothesize associations with every measurement point, check if they satisfy some constraints
//    std::cout << indent.str() << "Calculating cost of associating node " << current_node << std::endl;
//    std::cout << indent.str() << "parents_cost = " << parents_cost << std::endl;
    for ( int i = 0; i < measurement.points.size(); i++ )
    {
        geo::Vec3d cur_measurement_pt = measurement.points[i];
        double cur_measurement_std_dev = measurement.uncertainties[i];

        double local_cost = cost_calculator.calculateCost(*graph_ptr_, cur_measurement_pt, cur_measurement_std_dev, 0.05, graph_positions, level, associations, path_);

        double cost_so_far = parents_cost + local_cost;

//        std::cout << indent.str() << "with point " << cur_measurement_pt << std::endl;
//        std::cout << indent.str() << "local cost  = " << local_cost << std::endl;
//        std::cout << indent.str() << "cost_so_far = " << cost_so_far << std::endl;

        if ( local_cost <= max_no_std_devs &&
             local_cost != -1.0 &&                      // TODO: floating point equality check, not very elegant...
             cost_so_far < best_association_cost_)
        {

//            std::cout << indent.str() << "This is good enough!" << std::endl;
            Q.push(std::make_pair(local_cost,i));
        }
    }


    // Calculate consequences for the hypotheses that were good enough to put in the queue
    double best_cost = 100;
    AssociatedMeasurement input_associations = associations;

    while ( !Q.empty() )
    {
//        std::cout << indent.str() << "Considering node " << current_node << " for association with" << std::endl;

        // Get hypothesis with best cost from queue
        std::pair<double, int> hypothesis = Q.top();
        Q.pop();

        if ( hypothesis.second == -1 )
            hypothesis.first = 0.0;

        double cost_so_far = parents_cost + hypothesis.first;

//        if ( hypothesis.second == -1 )
//            std::cout << indent.str() << "no point (" << hypothesis.first << ")" << std::endl;
//        else
//            std::cout << indent.str() << "point " << measurement.points[hypothesis.second] << " (" << hypothesis.first << ")" << std::endl;

        if ( hypothesis.first > best_cost )
        {
//            std::cout << indent.str() << "this local cost is higher than best cost. Don't look further!" << std::endl;
            break;
        }

        // Create a copy of the measurement and associations to pass on
        Measurement reduced_measurement = measurement;
        AssociatedMeasurement prog_associations = input_associations;

        // If no association with a point...
        if ( hypothesis.second == -1 )
        {
            // ...penalize if this results in unassociated points further on
            int point_surplus = measurement.points.size() - (graph_positions.nodes.size() - (associations.nodes.size() + 1));
            if ( point_surplus > 0 )
            {
                double penalty = point_surplus * max_no_std_devs;
                if ( penalty + cost_so_far > best_association_cost_ )
                {
//                    std::cout << indent.str() << "Penalizing for leaving unassociated points: penalty + cost_so_far = " << penalty + cost_so_far << " > " << best_association_cost_ << std::endl;
                    continue;
                }
            }
        }
        else
        {
            // ...reduce by the locally hypothesized point...
            reduced_measurement.erase(hypothesis.second);

            // ...and add the association to the progressing associations.
            geo::Vec3d cur_measurement_pt = measurement.points[hypothesis.second];
            double cur_measurement_std_dev = measurement.uncertainties[hypothesis.second];

            prog_associations.append(cur_measurement_pt, cur_measurement_std_dev, current_node);
        }

        // Calculate the sum of the currently hypothesized association and the best associations resulting from the reduced problem
        double total_cost = hypothesis.first + associate( graph_positions, reduced_measurement, prog_associations, cost_calculator, max_no_std_devs, cost_so_far, level+1 );

//        std::cout << indent.str() << "total_cost = " << total_cost << std::endl;
//        std::cout << indent.str() << "best_cost = " << best_cost << std::endl;

        if ( total_cost - best_association_cost_ > 0.00001 )
        {
//            std::cout << indent.str() << "Not better than best association cost (" << best_association_cost_ << "), not storing resulting associations" << std::endl;
            continue;
        }

        // Store the total cost if it is better than we've seen before
        if ( total_cost < best_cost )
        {
//            std::cout << indent.str() << "Better than before, storing the resulting associations" << std::endl;
            best_cost = total_cost;
            associations = prog_associations;
        }
    }

    return best_cost;
}

// -----------------------------------------------------------------------------------------------

bool Associator::getAssociations( const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
{
    unassociated_points_ = measurement;
    best_association_cost_ = 100;

    // Find a path through the graph starting from the associated nodes
    PathFinder pathFinder( *graph_ptr_, associations.nodes );
    pathFinder.findPath( goal_node_i, path_ );

    // Calculate the positions of graph nodes on the path
    AssociatedMeasurement path_positions = associations;
    calculatePositions( *graph_ptr_, path_, path_positions );

    std::cout << "Path positions: \n[";
    for ( int i = 0; i < path_positions.nodes.size(); ++i )
    {
        std::cout << path_positions.nodes[i] << ": " << path_positions.measurement.points[i] << ", ";
    }
    std::cout << std::endl;

    associations.clear();
    associations.measurement.time_stamp = measurement.time_stamp;
    associations.measurement.frame_id = measurement.frame_id;

    if ( !costCalculators_.size() )
    {
        std::cout << "\033[31m" << "[ASSOCIATOR] GetAssociations: No cost calculator modules available!" << "\033[0m" << std::endl;
    }

    for ( int i = 0; i < costCalculators_.size(); ++i )
    {
        // Call the recursive association algorithms
        associate( path_positions, unassociated_points_, associations, *costCalculators_[i], max_assoc_dists_[i], 0.0);

        std::cout << associations.nodes.size() << " associations found after using costcalculator " << i << std::endl;

        // Reduce unassociated_points with any associated points and path_positions with the associated nodes before calling associate
        for ( int j = 0; j < associations.nodes.size(); ++j )
        {
            std::vector<geo::Vec3d>::iterator it = std::find( unassociated_points_.points.begin(),
                                                              unassociated_points_.points.end(),
                                                              associations.measurement.points[j] );
            if ( it != unassociated_points_.points.end() )
            {
                unassociated_points_.points.erase( it );
            }
        }
    }

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
