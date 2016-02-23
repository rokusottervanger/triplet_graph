cmake_minimum_required(VERSION 2.8.3)
project(triplet_graph)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geolib2
  tue_config
  rviz
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
 CATKIN_DEPENDS geolib2 tf tue_config
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


## BEGIN_TUTORIAL
## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
#qt4_wrap_cpp(MOC_FILES
#  include/triplet_graph/tools/rviz_tool.h
#)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  tools/rviz_tool.cpp
  include/triplet_graph/tools/moc_rviz_tool.cxx
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(triplet_graph_tools ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(triplet_graph_tools ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL



file(GLOB_RECURSE HEADER_FILES include/*.h)

# add_library(library_name
#     src/lib_source_file1.cpp
#     ...
# )
# target_link_libraries(library_name ${catkin_LIBRARIES})

add_library(triplet_graph
#    src/IncompleteRelation.cpp
    src/Path.cpp
    src/Measurement.cpp
    src/Visualizer.cpp
    src/Node.cpp
    src/Edge2.cpp
    src/Graph.cpp
    src/PathFinder.cpp
    src/graph_operations.cpp
    src/CornerDetector.cpp
    src/OdomTracker.cpp
    src/Associator.cpp
    ${HEADER_FILES}
)
target_link_libraries(triplet_graph
    ${catkin_LIBRARIES}
)

add_executable(mapping
    src/mapping.cpp
)
target_link_libraries(mapping triplet_graph ${catkin_LIBRARIES})

add_executable(localization
    src/localization.cpp
)
target_link_libraries(localization triplet_graph ${catkin_LIBRARIES})



