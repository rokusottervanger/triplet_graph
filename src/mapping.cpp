#include <triplet_graph/CornerDetector.h>
#include <triplet_graph/Graph.h>
#include <triplet_graph/graph_operations.h>
#include <triplet_graph/OdomTracker.h>
#include <triplet_graph/Measurement.h>
#include <triplet_graph/Visualizer.h>
#include <triplet_graph/Path.h>

#include <tue/profiling/timer.h>
#include <tue/config/configuration.h>

int main(int argc, char** argv)
{
    // - - - - - - - - - - - - - - - - - -
    // Initialize

    tue::Configuration config;

    ros::init(argc, argv, "laser_processor");

    // Parse arguments
    if ( argc < 2 )
    {
        std::cout << "Usage: \n\n        laser_processor MAPPING_CONFIG.yaml" << std::endl;
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


    // - - - - - - - - - - - - - - - - - -
    // Configure corner detection

    if ( config.readGroup("corner_detector") )
    {
        std::cout << "Configuring corner detector..." << std::endl;
        if ( !cornerDetector.configure(config) )
            return -1;
        std::cout << "Done!" << std::endl << std::endl;
        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "No config found for corner detector nor for a simulator" << "\033[0m" << std::endl;
        return -1;
    }


    // - - - - - - - - - - - - - - - - - -
    // Configure odom tracking

    if ( config.readGroup("odom_tracker") )
    {
        std::cout << "Configuring odom tracker..." << std::endl;
        odomTracker.configure(config);
        std::cout << "Done!" << std::endl << std::endl;
        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "[ODOM TRACKER] Configure: No configuration for odom tracker found!" << "\033[0m" << std::endl;
        return -1;
    }


    // - - - - - - - - - - - - - - - - - -
    // Configure visualization if set

    if ( config.readGroup("visualization") )
    {
        std::cout << "Configuring visualizer..." << std::endl;
        visualizer.configure(config);
        config.endGroup();
        std::cout << "Done!" << std::endl << std::endl;
    }


    // - - - - - - - - - - - - - - - - - -
    // Configure association

    double max_association_distance;

    if ( config.readGroup("association") )
    {
        config.value("max_association_distance", max_association_distance );
        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "[ODOM TRACKER] Configure: No configuration for association found!" << "\033[0m" << std::endl;
        return -1;
    }

    config.value("graph_filename",graph_filename);

    ros::Rate loop_rate(15);

    int target_node = -1;
    int loop = 0;

    // old_associations are always the latest associations that yield succesful localization
    triplet_graph::AssociatedMeasurement old_associations;
    bool localized = true;

    while (ros::ok())
    {

        // - - - - - - - - - - - - - - - - - -
        // Start loop timer

        std::cout << "Starting timer" << std::endl;
        tue::Timer timer;
        timer.start();
        std::cout << "Done" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Instantiate stuff

        triplet_graph::Measurement measurement;
        triplet_graph::Measurement unassociated_points;
        triplet_graph::AssociatedMeasurement associations;
        geo::Transform delta = geo::Transform::identity();
        triplet_graph::Path path;


        // - - - - - - - - - - - - - - - - - -
        // Find corners

        std::cout << "Detecting corners" << std::endl;
        cornerDetector.process(measurement);
        std::cout << measurement.points.size() << " corners detected" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Update position using odom data

        std::cout << "Getting odom delta" << std::endl;
        odomTracker.getDelta(delta,measurement.time_stamp);


        old_associations = delta.inverse() * old_associations;
        associations = old_associations;
        std::cout << "old_associations.size() = " << old_associations.nodes.size() << std::endl;
        std::cout << "Done" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Associate

        std::cout << "Trying to associate..." << std::endl;
        triplet_graph::associate( graph, measurement, associations, unassociated_points, target_node, path, max_association_distance);

        // Check if localization was succesful
        if ( associations.nodes.size() >= 2 )
        {
            std::vector<triplet_graph::Node> nodes = graph.getNodes();
            for ( std::vector<int>::iterator it_1 = associations.nodes.begin(); it_1 != associations.nodes.end(); ++it_1 )
            {
                for ( std::vector<int>::iterator it_2 = it_1+1 ; it_2 != associations.nodes.end(); ++it_2 )
                {
                    int num_of_common_trips = nodes[*it_1].tripletsByPeer(*it_2).size();
                    if ( num_of_common_trips > 0 )
                    {
                        localized = true;
                        std::cout << associations.nodes.size() << " associations found, state is: localized" << std::endl;
                        goto done;
                    }
                }
            }
            std::cout << "No common triplets found in " << associations.nodes.size() << " associations, state is: not localized" << std::endl;
            localized = false;
        }
        else
        {
            std::cout << associations.nodes.size() << " associations found, state is: not localized" << std::endl;
            localized = false;
        }
        done:

        std::cout << "path.size() = " << path.size() << std::endl;
        std::cout << "graph.size() = " << graph.size() << std::endl;

//        if ( loop > 1 && measurement.points.size() > 0 && graph.size() != path.size())
//            return -1;

        // If successful, store the associations for the next run
        if ( localized )
            old_associations = associations;

        std::cout << "old_associations' size after association: " << old_associations.nodes.size() << std::endl;

        // Proceed using latest associations that led to localization
        associations = old_associations;


        // - - - - - - - - - - - - - - - - - -
        // Update graph

        // Only necessary for new associated measurements
        if ( localized )
        {
            // Updates existing edges and adds edges between measured points
            std::cout << "Updating graph..." << std::endl;
            triplet_graph::updateGraph( graph, associations );
            std::cout << "Done!" << std::endl << std::endl;
        }


        // - - - - - - - - - - - - - - - - - -
        // Extend graph

        std::cout << "Extending graph with " << unassociated_points.points.size() << " nodes..." << std::endl;

        triplet_graph::extendGraph( graph, unassociated_points, associations );

        // extendgraph may add nodes to associations, so store those with the latest localizable associations.
        if ( localized || loop <= 2 )
            old_associations = associations;

        std::cout << "old_associations' size after extending: " << old_associations.nodes.size() << std::endl;

        std::cout << "Done!" << std::endl;
        loop ++;


        // - - - - - - - - - - - - - - - - - -
        // Visualize graph

        // TODO: Make function to easily visualize entire graph

        // Calculate positions again to visualize them in rviz:
//        std::vector<geo::Vec3d> positions(graph.size());
//        for ( int i = 0; i < associations.nodes.size(); ++i )
//            positions[associations.nodes[i]] = associations.measurement.points[i];

//        calculatePositions(graph, positions, path);

//        // Visualize positions
//        triplet_graph::Measurement vis_graph;
//        vis_graph.points = positions;
//        vis_graph.frame_id = measurement.frame_id;
//        vis_graph.time_stamp = measurement.time_stamp;

//        std::cout << "positions vector:" << std::endl;
//        for ( std::vector<geo::Vec3d>::iterator it = positions.begin(); it != positions.end(); ++it )
//            std::cout << *it << std::endl;
//        std::cout << std::endl;
//        std::cout << "Path: " << std::endl;
//        std::cout << path << std::endl;

//        visualizer.publish(vis_graph);


        // - - - - - - - - - - - - - - - - - -
        // Spin ros and sleep

        ros::spinOnce();

        std::cout << "Loop time: " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        std::cout << std::endl << "----------------------------------------------------------" << std::endl;

        loop_rate.sleep();
    }


    // - - - - - - - - - - - - - - - - - -
    // Save graph

    std::cout << "Writing graph config to disk..." << std::endl;
    triplet_graph::save(graph, graph_filename);
    std::cout << "Saved!" << std::endl;
}


