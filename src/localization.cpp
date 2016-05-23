#include "triplet_graph/CornerDetector.h"
#include "triplet_graph/Graph.h"
#include "triplet_graph/graph_operations.h"
#include "triplet_graph/OdomTracker.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Path.h"

#include <tue/profiling/timer.h>
#include <tue/config/configuration.h>

#include <csignal>

void signalHandler( int signum )
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    exit(signum);
}

int main(int argc, char** argv)
{
    // - - - - - - - - - - - - - - - - - -
    // Initialize

    tue::Configuration config;

    ros::init(argc, argv, "localization");

    // Parse arguments
    if ( argc < 2 )
    {
        std::cout << "Usage: \n\n        rosrun triplet_graph localization LOCALIZATION_CONFIG.yaml" << std::endl;
        return 1;
    }

    std::string config_filename = argv[1];
    config.loadFromYAMLFile(config_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load localization configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    triplet_graph::Graph graph;

    triplet_graph::CornerDetector cornerDetector;
    triplet_graph::OdomTracker odomTracker;
    triplet_graph::Visualizer visualizer;

    std::string sensor_frame_id;


    // - - - - - - - - - - - - - - - - - -
    // Configure corner detection

    if ( config.readGroup("corner_detector") )
    {
        std::cout << "Configuring corner detector..." << std::endl;
        if ( !cornerDetector.configure(config) )
            return -1;

        if ( !config.value("frame_id", sensor_frame_id) )
        {
            std::cout << "\033[31m" << "No frame_id found in corner_detector config" << "\033[0m" << std::endl;
            return -1;
        }

        std::cout << "Done!" << std::endl << std::endl;
        config.endGroup();
    }
    else
    {
        std::cout << "\033[31m" << "No config found for corner detector" << "\033[0m" << std::endl;
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
        std::cout << "\033[31m" << "No configuration for odom tracker found!" << "\033[0m" << std::endl;
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
    else
    {
        std::cout << "No visualizer configuration found" << std::endl;
    }


    // - - - - - - - - - - - - - - - - - -
    // Load and configure graph

    std::string graph_filename;
    if ( config.value("graph_filename",graph_filename) )
    {
        std::cout << "Loading graph from config file..." << std::endl;
        if ( !triplet_graph::load(graph, graph_filename) )
        {
            std::cout << "Failed to load graph" << std::endl;
            return -1;
        }
        std::cout << "Loaded!" << std::endl;
    }
    else
    {
        std::cout << "No graph_filename defined in config" << std::endl;
        return -1;
    }

    // - - - - - - - - - - - - - - - - - -
    // Configure initial pose

    bool localized;
    triplet_graph::AssociatedMeasurement old_associations;

    if ( config.readArray("initial_pose") )
    {
        while ( config.nextArrayItem() )
        {
            int node;
            config.value("node",node);

            double x,y;
            if ( config.readGroup("position") )
            {
                config.value("x",x);
                config.value("y",y);
                config.endGroup();
            }
            geo::Vec3d point(x,y,0.0);

            double std_dev;
            if ( !config.value("std_dev", std_dev) )
            {
                std::cout << "\033[31m" << "[LOCALIZATION]: Error while configuring initial pose. No std_dev given for initial point" << "\033[0m" << std::endl;
                return -1;
            }

            old_associations.append(point, std_dev, node);
        }
        old_associations.measurement.frame_id = sensor_frame_id;
        old_associations.measurement.time_stamp = ros::Time::now();
        config.endArray();
    }

    ros::Rate loop_rate(15);

    int target_node = -1;

    // old_associations are always the latest associations that yield succesful localization

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
        /* todo: use some odom error model. Don't just add it to the measurement error, because
         * measurement error is random for every point in a measurement, odom error is random
         * for every measurement, but constant over the points in one measurement!
         */

        old_associations = delta.inverse() * old_associations;
        associations = old_associations;
        std::cout << "old_associations.size() = " << old_associations.nodes.size() << std::endl;
        std::cout << "Done" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Associate

        std::cout << "Trying to associate..." << std::endl;
        triplet_graph::associate( graph, measurement, associations, unassociated_points, target_node, path, config );

        visualizer.publish(triplet_graph::generateVisualization(graph, old_associations, path));

        // Check if localization was succesful
        if ( associations.nodes.size() >= 2 )
        {
            triplet_graph::Graph::const_iterator node_it = graph.begin();

            for ( std::vector<int>::iterator it_1 = associations.nodes.begin(); it_1 != associations.nodes.end(); ++it_1 )
            {
                for ( std::vector<int>::iterator it_2 = it_1+1 ; it_2 != associations.nodes.end(); ++it_2 )
                {
                    node_it = graph.begin() + *it_1;
                    if ( node_it->deleted )
                    {
                        std::cout << "\033[31m" << "[localization] ERROR! Bug! One of the associated nodes is a deleted node. This is never supposed to happen!" << "\033[0m" << std::endl;
                        return -1;
                    }

                    int num_of_common_trips = node_it->tripletsByPeer(*it_2).size();

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

        signal(SIGINT, signalHandler);

        // If successful, store the associations for the next run
        if ( localized )
            old_associations = associations;

        std::cout << "old_associations' size after association: " << old_associations.nodes.size() << std::endl;

        // Proceed using latest associations that led to localization
        associations = old_associations;


        // - - - - - - - - - - - - - - - - - -
        // Spin ros and sleep

        std::cout << "Loop time: " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
        std::cout << std::endl << "----------------------------------------------------------" << std::endl;

        loop_rate.sleep();
        ros::spinOnce();

    }
}


