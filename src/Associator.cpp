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

double Associator::associate(const AssociatedMeasurement& graph_positions,
                             const Measurement& measurement,
                             AssociatedMeasurement& associations,
                             const CostCalculator& cost_calculator,
                             const double max_no_std_devs,
                             const double parents_cost=0.0,
                             int level=0)
{
    std::stringstream indent;
    while ( indent.str().size() < level*2 )
        indent << " ";

    AssociatedMeasurement input_associations = associations;


    // ------------------------------
    // ------------------------------
    // BASE CASE
    // ------------------------------
    // ------------------------------

    if ( measurement.points.size() == 0 )
    {
        associations.measurement.frame_id = measurement.frame_id;
        associations.measurement.time_stamp = measurement.time_stamp;
        std::cout << indent.str() << "Returning zero, because measurement is empty" << std::endl;
        if ( parents_cost < best_association_cost_ )
            best_association_cost_ = parents_cost;

        return 0.0;
    }
    if ( graph_positions.measurement.points.size() == 0 )
    {
        associations.measurement.frame_id = measurement.frame_id;
        associations.measurement.time_stamp = measurement.time_stamp;
        std::cout << indent.str() << "Returning #of remaining points: " << measurement.points.size() << " times max cost, because graph_positions is empty" << std::endl;

        double return_cost = measurement.points.size() * max_no_std_devs;
        double total_cost = return_cost + parents_cost;

        if ( total_cost < best_association_cost_ )
            best_association_cost_ = total_cost;

        return return_cost;
    }



    // ------------------------------
    // ------------------------------
    // RECURSIVE CASE
    // ------------------------------
    // ------------------------------

    // Take a node from the path to associate a measurement with
    AssociatedMeasurement reduced_graph_positions = graph_positions;
    reduced_graph_positions.erase(0);

    // Make priority queue of association hypotheses ordered by local cost
    std::priority_queue<std::pair<double,int>, std::vector< std::pair<double, int> >, std::less<std::pair<double, int> > > Q;

    // Hypothesize that no point associates with this node
    // Calculate the cost of not associating this node (if that leaves more points than nodes, at least one point must be unassociated)
    int point_surplus = measurement.points.size() - graph_positions.nodes.size();
    if ( point_surplus > 0 )
    {
        std::cout << indent.str() << "Foreseeing a point surplus of " << point_surplus << " if not associating" << std::endl;
    }

    Q.push(std::make_pair( std::max(point_surplus,0) * max_no_std_devs,-1) );
//    Q.push(std::make_pair(0.0,-1));

    std::cout << indent.str() << "Calculating costs for associating node " << graph_positions.nodes[0] << std::endl;

    // Hypothesize association with every measured point
    for ( int i = 0; i < measurement.points.size(); i++ )
    {
        geo::Vec3d cur_measurement_pt = measurement.points[i];
        double cur_measurement_std_dev = measurement.uncertainties[i];

        // Calculate cost of single hypothesized association
        double local_cost = cost_calculator.calculateCost(*graph_ptr_, cur_measurement_pt, cur_measurement_std_dev, graph_positions, 0, input_associations, path_);

        // TODO: HACK! if a triplet is inverted, it's rejected right away, but this is too
        // strict for 'flat' triangles. These may sometimes invert due to sensor noise.
        if ( local_cost == -1.0 )
            continue;

        // Only the hypotheses with a low enough local cost are added to the queue
        if ( local_cost <= max_no_std_devs )
        {
            std::cout << indent.str() << "with point " << cur_measurement_pt << " is good enough!" << std::endl;
            Q.push(std::make_pair(local_cost,i));
        }
    }


    // Calculate consequences for the hypotheses that were good enough to put in the queue
    double best_cost = 1e20;

    std::cout << indent.str() << "Considering node " << graph_positions.nodes[0] << " for association with" << std::endl;
    while ( !Q.empty() )
    {
        std::pair<double, int> hypothesis = Q.top();
        Q.pop();

        if ( hypothesis.second == -1 )
            std::cout << indent.str() << "no point (" << hypothesis.first << ")" << std::endl;
        else
            std::cout << indent.str() << "point " << measurement.points[hypothesis.second] << " (" << hypothesis.first << ")" << std::endl;

        double cost_so_far = parents_cost + hypothesis.first;

        std::cout << indent.str() << "cost_so_far = " << cost_so_far << std::endl;

        // If all hypothesized associations so far are more expensive than the best
        // association cost so far, further hypotheses are only more expensive, so
        // return this high cost, so that it is not used in higher generations of
        // the hypothesis tree.
        if ( cost_so_far > best_association_cost_ )
        {
            std::cout << indent.str() << "higher than best overall cost. Break!" << std::endl;
            break;
        }

        if ( hypothesis.first > best_cost )
        {
            std::cout << indent.str() << "local cost is higher than best further cost. Break!" << std::endl;
            break;
        }

        // create a copy of the measurement and associations
        Measurement reduced_measurement = measurement;
        AssociatedMeasurement prog_associations = input_associations;

        // If association with any point...
        if ( hypothesis.second != -1 )
        {
            geo::Vec3d cur_measurement_pt = measurement.points[hypothesis.second];
            double cur_measurement_std_dev = measurement.uncertainties[hypothesis.second];

            // ...reduce by the locally hypothesized point...
            reduced_measurement.erase(hypothesis.second);

            // ...and add the association to the progressing associations.
            prog_associations.append(cur_measurement_pt, cur_measurement_std_dev, graph_positions.nodes[0]);
        }

        // Calculate the sum of the currently hypothesized association and the best associations resulting from the reduced problem
        double total_cost = hypothesis.first + associate( reduced_graph_positions, reduced_measurement, prog_associations, cost_calculator, max_no_std_devs, cost_so_far, level+1 );

        std::cout << indent.str() << "total_cost = " << total_cost << std::endl;

        // Store the total cost if it is better than we've seen before
        if ( total_cost < best_cost )
        {
            std::cout << indent.str() << "Better than before, storing the resulting associations" << std::endl;
            best_cost = total_cost;
            associations = prog_associations;

            // If this is the first generation in the hypothesis tree, store it as the best total association cost.
            if ( level == 0 && best_cost < best_association_cost_ )
            {
                std::cout << indent.str() << "BEST TOTAL COST SO FAR!" << std::endl;
                best_association_cost_ = best_cost;
            }
        }
    }

    return best_cost;
}


// -----------------------------------------------------------------------------------------------

bool Associator::getAssociations( const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
{
    unassociated_points_ = measurement;
    best_association_cost_ = 1e308;

    // Find a path through the graph starting from the associated nodes
    PathFinder pathFinder( *graph_ptr_, associations.nodes );
    pathFinder.findPath( goal_node_i, path_ );

    // Calculate the positions of graph nodes on the path
    AssociatedMeasurement path_positions = associations;
    calculatePositions( *graph_ptr_, path_, path_positions );

    associations.clear();

    if ( !costCalculators_.size() )
    {
        std::cout << "\033[31m" << "[ASSOCIATOR] GetAssociations: No cost calculator modules available!" << "\033[0m" << std::endl;
    }

    for ( int i = 0; i < costCalculators_.size(); ++i )
    {
        // Reduce unassociated_points with any associated points and path_positions with the associated nodes before calling associate
        for ( int j = 0; j < associations.nodes.size(); ++j )
        {
            path_positions.erase( path_positions.node_indices[ associations.nodes[j] ] );
            unassociated_points_.points.erase( std::find(unassociated_points_.points.begin(), unassociated_points_.points.end(), associations.measurement.points[i] ) );
        }

        // Call the recursive association algorithms
        associate( path_positions, unassociated_points_, associations, *costCalculators_[i], max_assoc_dists_[i]);

        std::cout << associations.nodes.size() << " associations found after using costcalculator " << i << std::endl;
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
