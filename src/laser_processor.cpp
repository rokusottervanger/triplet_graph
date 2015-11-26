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
    config.value("graph_filename",graph_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load laser processor configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    triplet_graph::Graph graph;
    triplet_graph::CornerDetector cornerDetector;
    triplet_graph::OdomTracker odomTracker;

    triplet_graph::AssociatedMeasurement associations;

    // TODO: add config for odomtracker to config file

    std::cout << "Configuring corner detector..." << std::endl;
    cornerDetector.configure(config);
    std::cout << "Done!" << std::endl << std::endl;

    std::cout << "Configuring odom tracker..." << std::endl;
    odomTracker.configure(config);
    std::cout << "Done!" << std::endl << std::endl;

    ros::Rate loop_rate(15);

    int target_node = -1;

    while (ros::ok())
    {
        std::cout << "Starting timer" << std::endl;
        tue::Timer timer;
        timer.start();
        std::cout << "Done" << std::endl << std::endl;

        triplet_graph::Measurement measurement;
        geo::Transform delta;

        std::cout << "Detecting corners" << std::endl;
        cornerDetector.process(measurement);
        std::cout << measurement.points.size() << " corners detected" << std::endl << std::endl;

        std::cout << "Getting odom delta" << std::endl;
        odomTracker.getDelta(delta,measurement.time_stamp);
        std::cout << "Got odom data: " << delta << std::endl << std::endl;

        std::cout << "Trying to associate..." << std::endl;
        triplet_graph::associate( graph, measurement, associations, delta, target_node );
        std::cout << "Associated " << associations.nodes.size() << " nodes" << std::endl << std::endl;

        std::cout << "Updating graph..." << std::endl;
        triplet_graph::updateGraph( graph, associations );
        std::cout << "Done!" << std::endl << std::endl;

        std::cout << "Extending graph with " << measurement.points.size() - associations.nodes.size() << " nodes..." << std::endl;
        triplet_graph::extendGraph( graph, measurement, associations );
        std::cout << "Done!" << std::endl;

        ros::spinOnce();

        std::cout << "Loop time: " << timer.getElapsedTimeInMilliSec() << std::endl;

        loop_rate.sleep();
    }
    std::cout << "Writing graph config to disk..." << std::endl;
    triplet_graph::save(graph, graph_filename);
    std::cout << "Saved!" << std::endl;
}


