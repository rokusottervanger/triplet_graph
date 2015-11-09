#include "triplet_graph/CornerDetector.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/Visualizer.h"

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
    config.loadFromYAMLFile(config_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load laser processor configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    triplet_graph::CornerDetector cornerDetector;

    cornerDetector.configure(config);

    ros::Rate loop_rate(15);

    while (ros::ok())
    {
        triplet_graph::Measurement measurement;

        cornerDetector.process(measurement);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
