#include "triplet_graph/CornerDetector.h"
#include "triplet_graph/Graph.h"
#include "triplet_graph/graph_operations.h"
#include "triplet_graph/OdomTracker.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Path.h"
#include "triplet_graph/PathFinder.h"

#include <tue/profiling/timer.h>
#include <tue/config/configuration.h>

#include <csignal>
#include <fstream>

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

    bool experiment = false;
    if ( argc > 2 )
    {
        std::string exp_arg = argv[2];
        if ( exp_arg == "--experiment" )
        {
            experiment = true;
        }
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

    int target_node;
    if ( !config.value("target_node",target_node) )
        target_node = -1;

    ros::Rate loop_rate(15);

    // old_associations are always the latest associations that yield succesful localization

    triplet_graph::AssociatedMeasurement stored_associations = old_associations;

    std::ofstream output_file;
    if ( experiment )
    {
        output_file.open ("data.csv");
    }

    ros::Time start_time = ros::Time::now();

    while (ros::ok())
    {

        // - - - - - - - - - - - - - - - - - -
        // Start loop timer

        std::cout << "Starting timer" << std::endl;
        tue::Timer timer;
        timer.start();
        std::cout << "Done" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Find corners

        std::cout << "Detecting corners" << std::endl;
        triplet_graph::Measurement measurement;
        cornerDetector.process(measurement);
        std::cout << measurement.points.size() << " corners detected" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Update position using odom data

        std::cout << "Getting odom delta" << std::endl;
        geo::Transform delta = geo::Transform::identity();
        odomTracker.getDelta(delta, measurement.time_stamp);
        /* todo: use some odom error model. Don't just add it to the measurement error, because
         * measurement error is random for every point in a measurement, odom error is random
         * for every measurement, but constant over the points in one measurement!
         */

        old_associations = delta.inverse() * old_associations;
        triplet_graph::AssociatedMeasurement associations;
        associations = old_associations;
        std::cout << "old_associations.size() = " << old_associations.nodes.size() << std::endl;
        std::cout << "Done" << std::endl << std::endl;


        // - - - - - - - - - - - - - - - - - -
        // Associate

        std::cout << "Trying to associate..." << std::endl;
        triplet_graph::Measurement unassociated_points;
        triplet_graph::Path path;
        localized = triplet_graph::associate( graph, measurement, associations, unassociated_points, -1, path, config );

        triplet_graph::AssociatedMeasurement visualization_measurement;

        // If succesful, store associations for the next run (one that will get odom update, one that will not) and visualize the graph
        if ( localized )
        {
            visualization_measurement = triplet_graph::generateVisualization(graph, old_associations, path);
            visualizer.publish(visualization_measurement);
            stored_associations = associations;
            old_associations = associations;

        }
        // If not succesful, visualize the graph using the last made associations without odom update
        else
        {
            visualization_measurement = triplet_graph::generateVisualization(graph, stored_associations, path);
            visualizer.publish(visualization_measurement);
        }

        if ( target_node != -1 )
        {
            // For experiments!!! begin
            if ( experiment )
            {
                geo::Vec3d point = visualization_measurement.measurement.points[visualization_measurement.node_indices[target_node]];
                geo::Vec3d point_gt;
                geo::Vec3d point_amcl;
                double a = (507.0-552.0)*0.025;
                double b = (643.0-531.0)*0.025;
                geo::Vec3d point_in_odom_frame = geo::Vec3d(a,b,0.0);
                geo::Vec3d zero(0.0,0.0,0.0);
                geo::Vec3d robot_pos_gt;
                geo::Vec3d robot_pos_amcl;
                odomTracker.transformVector("/amigo/base_link", "/amigo/odom", zero, robot_pos_gt, measurement.time_stamp );
                odomTracker.transformVector("/amigo/base_link", "/map", zero, robot_pos_amcl, measurement.time_stamp );
                odomTracker.transformVector("/amigo/odom", "/amigo/base_laser", point_in_odom_frame, point_gt, measurement.time_stamp );
                odomTracker.transformVector("/map", "/amigo/base_laser", point_in_odom_frame, point_amcl, measurement.time_stamp );

                output_file << (measurement.time_stamp - start_time).toSec() << ", "
                            << point_gt.x << ", " << point_gt.y << ", "
                            << point.x << ", " << point.y << ", "
                            << point_amcl.x << ", " << point_amcl.y << ", "
                            << robot_pos_gt.x << ", " << robot_pos_gt.y << ", "
                            << robot_pos_amcl.x << ", " << robot_pos_amcl.y << ", "
                            << "\n";
            }
            // end for experiments
        }

//        signal(SIGINT, signalHandler);

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

    if (experiment)
    {
        std::cout << "Closing output file" << std::endl;
        output_file.close();
    }
}


