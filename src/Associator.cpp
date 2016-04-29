#include "triplet_graph/Associator.h"

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

double Associator::associate(const AssociatedMeasurement& graph_positions,
                              const Measurement& measurement,
                              AssociatedMeasurement& associations,
                              const CostCalculator& cost_calculator,
                              const double max_no_std_devs )
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
    Measurement reduced_measurement = measurement;
    reduced_measurement.points.pop_back();
    reduced_measurement.uncertainties.pop_back();


    // Hypothesize that the measurement point does not associate at all
    // ------------------------------

    // This sets the threshold for association
    double local_cost = max_no_std_devs;

    // Copy all graph positions (because nothing was associated, all of them are passed to the next recursion)
    AssociatedMeasurement reduced_graph_positions = graph_positions;
    AssociatedMeasurement hyp_associations = input_associations;

    // Calculate further associations and set this association and its cost as the benchmark for other associations
    double best_cost = local_cost + associate( reduced_graph_positions, reduced_measurement, hyp_associations, cost_calculator, max_no_std_devs );


    // Hypothesize association with every graph node
    // ------------------------------

    int best_node = -1;

    for ( int i = 0; i < graph_positions.measurement.points.size(); i++ )
    {

        // Calculate cost of hypothesized association
        // ------------------------------
        // Calculate local cost using most recent parent positions (use position from associations if possible, otherwise use calculated position from graph_positions)

        local_cost = cost_calculator.calculateCost(*graph_ptr_, cur_measurement_pt, cur_measurement_std_dev, graph_positions, i, input_associations, path_);

        if ( local_cost == -1.0 )
            continue;

        // Only if local cost is lower than the threshold for the total cost, proceed with further associations
        if ( local_cost < max_no_std_devs )
        {

            // Add association to progressing hypothesis (which is passed on to further recursions)
            AssociatedMeasurement prog_associations = input_associations;

            prog_associations.measurement.points.push_back(cur_measurement_pt);
            prog_associations.measurement.uncertainties.push_back(cur_measurement_std_dev);
            prog_associations.node_indices[graph_positions.nodes[i]] = prog_associations.nodes.size();
            prog_associations.nodes.push_back(graph_positions.nodes[i]);


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
            double total_cost = local_cost + associate( reduced_graph_positions, reduced_measurement, prog_associations, cost_calculator, max_no_std_devs );

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

bool Associator::getAssociations( const Measurement& measurement, AssociatedMeasurement& associations, const int goal_node_i )
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

    for ( int i = 1; i <= path_.size(); ++i )
    {
        // Calculate index in path
        int index = path_.size()-i; // Assumes order in path!!!!!!!

        path_positions.node_indices[path_[index]] = path_positions.nodes.size();
        path_positions.nodes.push_back(path_[index]);
        path_positions.measurement.points.push_back(positions[path_[index]]);
        path_positions.measurement.uncertainties.push_back(path_.costs[index]);
    }

    if ( !costCalculators_.size() )
    {
        std::cout << "\033[31m" << "[ASSOCIATOR] GetAssociations: No cost calculator modules available!" << "\033[0m" << std::endl;
    }
//    for ( std::vector<boost::shared_ptr<CostCalculator> >::const_iterator it = costCalculators_.begin(); it != costCalculators_.end(); ++it )
    for ( int i = 0; i < costCalculators_.size(); ++i )
    {
        // Call the recursive association algorithms
        associate( path_positions, measurement, associations, *costCalculators_[i], max_assoc_dists_[i]);
        // TODO: reduce measurement with the associated points path positions with the associated nodes before calling associate again.
    }

    associated_ = true;

    std::cout << "number of (recursive) function calls: " << calls_ << std::endl;

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
