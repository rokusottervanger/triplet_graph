cmake_minimum_required(VERSION 2.8.3)
project(triplet_graph)

find_package(catkin REQUIRED COMPONENTS
  geolib2
  tue_config
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES triplet_graph
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE HEADER_FILES include/*.h)

# add_library(library_name
#     src/lib_source_file1.cpp
#     ...
# )
# target_link_libraries(library_name ${catkin_LIBRARIES})

add_library(triplet_graph
#    src/IncompleteRelation.cpp
    src/Path.cpp
#    src/Measurement.cpp
    src/Node.cpp
    src/Graph.cpp
    src/graph_operations.cpp
    ${HEADER_FILES}
)
target_link_libraries(triplet_graph ${catkin_LIBRARIES})


#add_executable(rviz_publisher
#    src/rviz_publisher.cpp
#)
#target_link_libraries(rviz_publisher ${catkin_LIBRARIES})



