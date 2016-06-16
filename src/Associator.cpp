#include "triplet_graph/Associator.h"

#include <queue>

#include "triplet_graph/Graph.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/graph_operations.h"
#include "triplet_graph/PathFinder.h"
#include "triplet_graph/EdgeTensionPC.h"
#include "triplet_graph/NearestNeighborPC.h"

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
                    boost::shared_ptr<NearestNeighborPC> probCalculator(new NearestNeighborPC);
                    probCalculators_.push_back(probCalculator);

                    double min_association_prob = 0.0;
                    if ( !config.value("min_association_prob", min_association_prob) )
                        std::cout << "[ASSOCIATOR] Configure: Warning! No min_association_prob defined for " << module_type << " cost calculator module" << std::endl;

                    min_assoc_probs_.push_back(min_association_prob);
                }
                else if ( module_type == "edge_tension" )
                {
                    boost::shared_ptr<EdgeTensionPC> costCalculator(new EdgeTensionPC);
                    probCalculators_.push_back(costCalculator);

                    double min_association_prob = 0.0;
                    if ( !config.value("min_association_prob", min_association_prob) )
                        std::cout << "[ASSOCIATOR] Configure: Warning! No min_association_prob defined for " << module_type << " cost calculator module" << std::endl;

                    min_assoc_probs_.push_back(min_association_prob);
                }
                else
                {
                    std::cout << "[ASSOCIATOR] Configure: Warning! Unknown association cost module type given in config" << std::endl;
                }
            }
            config.endArray();

            if ( !probCalculators_.size() )
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

double intPow(double base, unsigned int exponent)
{
    if ( exponent == 0 )
        return 1.0;

    if ( exponent < 0 )
        return 1/intPow(base, -exponent);

    if ( exponent%2 == 0 )
    {
        double sq_rt = intPow(base,exponent/2);
        return sq_rt*sq_rt;
    }

    if ( exponent%2 == 1 )
        return base * intPow(base,exponent-1);

    std::cout << "THIS SHOULD NEVER HAPPEN!" << std::endl;
    return 0.0;
}

// -----------------------------------------------------------------------------------------------

// TODO's:
// - Check for a point surplus when not associating and penalize appropriately


double Associator::associate(const AssociatedMeasurement& graph_positions,
                             const Measurement& measurement,
                             AssociatedMeasurement& associations,
                             const ProbabilityCalculator& cost_calculator,
                             const double min_association_prob,
                             const double parents_prob=1.0,
                             int level=0)
{
    std::stringstream indent;
    while ( indent.str().size() < level*2 )
        indent << " ";

    // BASE CASE
    // ------------------------------

    if ( level == graph_positions.nodes.size() )
    {
//        std::cout << indent.str() << "Base case: level == graph_positions.nodes.size() " << std::endl;
        double return_prob = intPow(min_association_prob, measurement.points.size()); // TODO: rename min_association_prob
        double total_prob = return_prob * parents_prob;

        if ( total_prob > best_association_prob_ )
        {
            best_association_prob_ = total_prob;
        }

//        std::cout << indent.str() << "Returning " << return_prob << ", because " << measurement.points.size() << " points left" << std::endl;
        return return_prob;
    }
    if ( measurement.points.size() == 0 )
    {
//        std::cout << indent.str() << "Returning 1.0, because measurement is empty" << std::endl;
        if ( parents_prob > best_association_prob_ )
        {
            best_association_prob_ = parents_prob;
        }
        return 1.0;
    }


    // RECURSIVE CASE
    // ------------------------------

    int current_node = graph_positions.nodes[level];

    // If current node is already in the associated nodes, directly call next recursion for next node
    if ( associations.node_indices.find(current_node) != associations.node_indices.end() )
    {
        return associate( graph_positions, measurement, associations, cost_calculator, min_association_prob, parents_prob, level+1 );
    }

    std::priority_queue<std::pair<double,int>, std::vector< std::pair<double, int> >, std::less<std::pair<double, int> > > Q;

    // Hypothesize no association
    Q.push(std::make_pair(min_association_prob,-1));

    // Hypothesize associations with every measurement point, check if they satisfy some constraints
//    std::cout << indent.str() << "Calculating cost of associating node " << current_node << std::endl;
//    std::cout << indent.str() << "parents_prob = " << parents_prob << std::endl;
    for ( int i = 0; i < measurement.points.size(); i++ )
    {
        geo::Vec3d cur_measurement_pt = measurement.points[i];
        double cur_measurement_std_dev = measurement.uncertainties[i];

        double local_prob = cost_calculator.calculateProbability(*graph_ptr_, cur_measurement_pt, cur_measurement_std_dev, 0.05, graph_positions, level, associations, path_);

        double prob_so_far = parents_prob * local_prob;

//        std::cout << indent.str() << "with point " << cur_measurement_pt << std::endl;
//        std::cout << indent.str() << "local prob  = " << local_prob << std::endl;
//        std::cout << indent.str() << "prob_so_far = " << prob_so_far << std::endl;

        if ( local_prob >= min_association_prob &&
             prob_so_far > best_association_prob_)
        {

//            std::cout << indent.str() << "This is good enough!" << std::endl;
            Q.push(std::make_pair(local_prob,i));
        }
    }


    // Calculate consequences for the hypotheses that were good enough to put in the queue
    double best_prob = intPow(min_association_prob, measurement.points.size());
    AssociatedMeasurement input_associations = associations;

    while ( !Q.empty() )
    {
//        std::cout << indent.str() << "Considering node " << current_node << " for association with" << std::endl;

        // Get hypothesis with best probability from queue
        std::pair<double, int> hypothesis = Q.top();
        Q.pop();

        if ( hypothesis.second == -1 )
            hypothesis.first = 1.0;

        double prob_so_far = parents_prob * hypothesis.first;

//        if ( hypothesis.second == -1 )
//            std::cout << indent.str() << "no point (" << hypothesis.first << ")" << std::endl;
//        else
//            std::cout << indent.str() << "point " << measurement.points[hypothesis.second] << " (" << hypothesis.first << ")" << std::endl;

        if ( hypothesis.first < best_prob )
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
                double penalty = intPow(min_association_prob, point_surplus);
                if ( penalty * prob_so_far < best_association_prob_ )
                {
//                    std::cout << indent.str() << "Penalizing for leaving unassociated points: penalty + prob_so_far = " << penalty * prob_so_far << " > " << best_association_prob_ << std::endl;
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
        double total_prob = hypothesis.first * associate( graph_positions, reduced_measurement, prog_associations, cost_calculator, min_association_prob, prob_so_far, level+1 );

//        std::cout << indent.str() << "total_prob = " << total_prob << std::endl;
//        std::cout << indent.str() << "best_prob = " << best_prob << std::endl;

        if ( total_prob / best_association_prob_ < 0.9999999 )
        {
//            std::cout << indent.str() << "Not better than best association prob (" << best_association_prob_ << "), not storing resulting associations" << std::endl;
            continue;
        }

        // Store the total prob if it is better than we've seen before
        if ( total_prob > best_prob )
        {
//            std::cout << indent.str() << "Better than before, storing the resulting associations" << std::endl;
            best_prob = total_prob;
            associations = prog_associations;
        }
    }

    return best_prob;
}

// -----------------------------------------------------------------------------------------------

bool Associator::getAssociations( const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
{
    unassociated_points_ = measurement;

    // Find a path through the graph starting from the associated nodes
    PathFinder pathFinder( *graph_ptr_, associations.nodes );
    pathFinder.findPath( goal_node_i, path_ );

    // Calculate the positions of graph nodes on the path
    AssociatedMeasurement path_positions = associations;
    calculatePositions( *graph_ptr_, path_, path_positions );

    associations.clear();
    associations.measurement.time_stamp = measurement.time_stamp;
    associations.measurement.frame_id = measurement.frame_id;

    if ( !probCalculators_.size() )
    {
        std::cout << "\033[31m" << "[ASSOCIATOR] GetAssociations: No cost calculator modules available!" << "\033[0m" << std::endl;
    }

    for ( int i = 0; i < probCalculators_.size(); ++i )
    {
        // Worst case scenario is that no points are associated, so this will be the initial benchmark.
        best_association_prob_ = intPow(min_assoc_probs_[i],unassociated_points_.points.size()-2);

        // Call the recursive association algorithms
        associate( path_positions, unassociated_points_, associations, *probCalculators_[i], min_assoc_probs_[i], 1.0);

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
