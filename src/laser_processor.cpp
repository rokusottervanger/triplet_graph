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
    cornerDetector.configure(config);
    odomTracker.configure(config);

    ros::Rate loop_rate(15);

    int target_node = -1;

    while (ros::ok())
    {
        tue::Timer timer;
        timer.start();

        triplet_graph::Measurement measurement;
        geo::Transform delta;

        cornerDetector.process(measurement);

        odomTracker.getDelta(delta,measurement.time_stamp);

        triplet_graph::associate( graph, measurement, associations, delta, target_node ); // TODO: There is no target node when exploring, right?

        triplet_graph::updateGraph( graph, associations );

        triplet_graph::extendGraph( graph, measurement, associations );

        ros::spinOnce();

        std::cout << "Loop time: " << timer.getElapsedTimeInMilliSec() << std::endl;

        loop_rate.sleep();
    }
    std::cout << "Writing graph config to disk..." << std::endl;
    triplet_graph::save(graph, graph_filename);
    std::cout << "Saved!" << std::endl;
}


