#include "triplet_graph/CornerDetector.h"
#include "triplet_graph/Graph.h"
#include "triplet_graph/graph_operations.h"
#include "triplet_graph/OdomTracker.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Path.h"
#include <tue/profiling/timer.h>

int main(int argc, char** argv)
{
    // - - - - - - - - - - - - - - - - - -
    // Initialize

    tue::Configuration config;

    ros::init(argc, argv, "laser_processor");

    // Parse arguments
    if ( argc < 2 )
    {
        std::cout << "Usage: \n\n        laser_processor LASER_PROCESSOR_CONFIG.yaml" << std::endl;
        return 1;
    }

    std::string config_filename = argv[1];
    std::string graph_filename;
    config.loadFromYAMLFile(config_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load laser processor configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    triplet_graph::Graph graph;
    triplet_graph::CornerDetector cornerDetector;
    triplet_graph::OdomTracker odomTracker;
    triplet_graph::Visualizer visualizer;
    triplet_graph::AssociatedMeasurement associations;


    // - - - - - - - - - - - - - - - - - -
    // Configure everything

    std::cout << "Configuring corner detector..." << std::endl;
    cornerDetector.configure(config);
    std::cout << "Done!" << std::endl << std::endl;

    std::cout << "Configuring odom tracker..." << std::endl;
    odomTracker.configure(config);
    std::cout << "Done!" << std::endl << std::endl;

    if ( config.readGroup("visualization") )
    {
        std::cout << "Configuring visualizer..." << std::endl;
        visualizer.configure(config);
        config.endGroup();
        std::cout << "Done!" << std::endl << std::endl;
    }

    config.value("graph_filename",graph_filename);

    ros::Rate loop_rate(15);

    int target_node = -1;

    while (ros::ok())
    {

        // - - - - - - - - - - - - - - - - - -
        // Start loop timer

        std::cout << "Starting timer" << std::endl;
        tue::Timer timer;
        timer.start();
        std::cout << "Done" << std::endl << std::endl;

        triplet_graph::Measurement measurement;
        geo::Transform delta;
        triplet_graph::Path path;


        // - - - - - - - - - - - - - - - - - -
        // Find corners

        std::cout << "Detecting corners" << std::endl;
        cornerDetector.process(measurement);
        std::cout << measurement.points.size() << " corners detected" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Get odom data

        std::cout << "Getting odom delta" << std::endl;
        odomTracker.getDelta(delta,measurement.time_stamp);
        std::cout << "Got odom data: " << delta << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Associate

        std::cout << "Trying to associate..." << std::endl;
        triplet_graph::associate( graph, measurement, associations, delta, target_node, path );
        std::cout << "Associated " << associations.nodes.size() << " nodes" << std::endl << std::endl;
        if ( associations.nodes.size() > 1 )
        {
            graph.setAssociations(associations);
        }


        // - - - - - - - - - - - - - - - - - -
        // Update graph

        // Updates existing edges and adds edges between measured points
        std::cout << "Updating graph..." << std::endl;
        triplet_graph::updateGraph( graph, associations );
        std::cout << "Done!" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Extend graph

        int graph_size = graph.size();
        int nodes_to_add = measurement.points.size() - associations.nodes.size();

        std::cout << "Extending graph with " << nodes_to_add << " nodes..." << std::endl;

        triplet_graph::extendGraph( graph, measurement, associations );

        if ( graph.size() - graph_size != nodes_to_add )
            std::cout << "\033[31m" << "[LASER PROCESSOR] Graph size was " << graph_size << " and is now " << graph.size() << " but there were only " << nodes_to_add << " nodes unassociated!" << "\033[0m" << std::endl;
        std::cout << "Done!" << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Visualize graph

        // Calculate positions again to visualize them in rviz:
        std::vector<geo::Vec3d> positions(graph.size());
        for ( int i = 0; i < associations.nodes.size(); ++i )
            positions[associations.nodes[i]] = associations.measurement.points[i];

        calculatePositions(graph, positions, path);

        // Visualize positions
        triplet_graph::Measurement vis_graph;
        vis_graph.points = positions;
        vis_graph.frame_id = measurement.frame_id;
        vis_graph.time_stamp = measurement.time_stamp;

        std::cout << "positions vector:" << std::endl;
        for ( std::vector<geo::Vec3d>::iterator it = positions.begin(); it != positions.end(); ++it )
            std::cout << *it << std::endl;
        std::cout << std::endl;
        std::cout << "Path: " << std::endl;
        std::cout << path << std::endl;

        visualizer.publish(vis_graph);


        // - - - - - - - - - - - - - - - - - -
        // Spin ros and sleep

        ros::spinOnce();

        std::cout << "Loop time: " << timer.getElapsedTimeInMilliSec() << std::endl;

        std::cout << std::endl << "----------------------------------------------------------" << std::endl;

        loop_rate.sleep();
    }


    // - - - - - - - - - - - - - - - - - -
    // Save graph

    std::cout << "Writing graph config to disk..." << std::endl;
    triplet_graph::save(graph, graph_filename);
    std::cout << "Saved!" << std::endl;
}


