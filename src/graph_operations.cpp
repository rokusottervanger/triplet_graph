#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <fstream>

#include <geolib/datatypes.h>

#include "triplet_graph/graph_operations.h"
#include "triplet_graph/Graph.h"
#include "triplet_graph/Path.h"
#include "triplet_graph/Measurement.h"
#include "triplet_graph/PathFinder.h"
#include "triplet_graph/Visualizer.h"
#include "triplet_graph/Associator.h"

namespace triplet_graph
{

// -----------------------------------------------------------------------------------------------

int findNodeByID(const Graph& g, const std::string &id)
/**
 * Find the index of a node in the graph, given the node id
 */
{
    for ( Graph::const_iterator it = g.begin(); it != g.end(); ++it )
    {
        if ((*it).id == id)
//            return (it - g.begin());
            return it.getIndex(g);
    }

    // Node not found, return -1!
    return -1;
}

// -----------------------------------------------------------------------------------------------

bool configure(Graph& g, tue::Configuration &config)
/**
 * Configure a graph using a tue::Configuration
 */
{
    if ( config.readArray("nodes") )
    {
        while ( config.nextArrayItem() )
        {
            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled;
            if ( config.value("enabled", enabled, tue::OPTIONAL) && !enabled )
                continue;

            std::string id;
            if ( !config.value("id", id) )
            {
                std::cout << "\033[31m" << "[GRAPH] ERROR! Node config has no id" << "\033[0m" << std::endl;
                continue;
            }
            else
                g.addNode(id);
        }
        config.endArray();
    }
    else
    {
        std::cout << "[GRAPH] ERROR! Array 'nodes' not found" << std::endl;
        return false;
    }

    if ( config.readArray("edges") )
    {
        while ( config.nextArrayItem() )
        {
            int n1, n2;
            std::string id1, id2;

            if ( config.value("n1", id1) && config.value("n2", id2))
            {
                n1 = findNodeByID(g,id1);
                n2 = findNodeByID(g,id2);
            }
            else
            {
                std::cout << "\033[31m" << "[GRAPH] WARNING! No node ids found in edge" << "\033[0m" << std::endl;
                continue;
            }

            if ( n1 == -1 || n2 == -1 )
                std::cout << "\033[31m" << "[GRAPH] WARNING! Could not find nodes corresponding to edge" << "\033[0m" << std::endl;
            else
            {
                double length;
                if ( config.value("length", length, tue::REQUIRED) )
                {
                    double std_dev = 0.1;
                    if ( !config.value("std_dev", std_dev, tue::REQUIRED) )
                    {
                        std::cout << "[GRAPH] WARNING! No edge std dev defined in config. You're probably using an old config file. Using " << std_dev << " % of edge length as default" << std::endl;
                    }
                    g.addEdge2(n1,n2,length, std_dev);
                }
                else
                {
                    std::cout << "\033[31m" << "[GRAPH] WARNING Edge length not defined" << "\033[0m" << std::endl;
                    continue;
                }
            }
        }
        config.endArray();
    }
    else
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! No edges defined in config" << "\033[0m" << std::endl;
        return false;
    }

    if ( config.readArray("triplets") )
    {
        while(config.nextArrayItem())
        {
            int n1, n2, n3;
            std::string id1, id2, id3;
            if ( config.value("n1", id1) && config.value("n2", id2) && config.value("n3", id3) )
            {
                n1 = findNodeByID(g,id1);
                n2 = findNodeByID(g,id2);
                n3 = findNodeByID(g,id3);
            }
            else
            {
                std::cout << "\033[31m" << "[GRAPH] WARNING! No node ids found in triplet" << "\033[0m" << std::endl;
                continue;
            }

            if ( n1 == -1 || n2 == -1 || n3 == -1 )
                std::cout << "\033[31m" << "[GRAPH] WARNING! Could not find nodes corresponding to edge" << "\033[0m" << std::endl;
            else
                g.addEdge3(n1,n2,n3);
        }
    }
    else
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! No triplets defined in config" << "\033[0m" << std::endl;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------------------------

// TODO: check connectivity of graph before returning true!
bool load(Graph &graph, std::string filename)
/**
 * Utility function to load a graph configuration given the filename of the config file.
 */
{
    tue::Configuration config;
    if ( !config.loadFromYAMLFile(filename) )
    {
        std::cout << "[LOAD] config with filename " << filename << " cannot be loaded: " << config.error();
        return false;
    }

    return configure(graph,config);
}

// -----------------------------------------------------------------------------------------------

void setRigidEdges(Graph &graph, const std::vector<int>& nodes)
{
    for ( std::vector<int>::const_iterator it1 = nodes.begin(); it1 != nodes.end(); ++it1 )
    {
        for ( std::vector<int>::const_iterator it2 = nodes.begin(); it2 != it1; ++it2 )
        {
            graph.setEdgeRigid(*it1,*it2);
        }
    }
}

// -----------------------------------------------------------------------------------------------

void mergeNodes(Graph &graph, const std::vector<int>& nodes)
{
    int n1 = nodes.front();
    for ( std::vector<int>::const_iterator it = nodes.begin()+1; it != nodes.end(); ++it )
    {
        graph.mergeNodes(n1,*it);
    }
}

// -----------------------------------------------------------------------------------------------

void deleteNodes(Graph &graph, const std::vector<int>& nodes)
{
    for ( std::vector<int>::const_iterator it = nodes.begin(); it != nodes.end(); ++it )
    {
        graph.deleteNode(*it);
    }
}

// -----------------------------------------------------------------------------------------------

void calculatePositions(const Graph &graph, const Path& path, AssociatedMeasurement& positions)
/**
 * Given a graph, a path and a sparse vector of positions, calculates the positions of
 * all nodes in the path. The vector of positions must be of the same size as the graph
 * and must contain at least the positions of the root nodes of the given path.
 * TODO: This should really be cleaned up by using exceptions in graph methods instead
 * of returning -1 and having to check for that all the time...
 */
{
    // Calculate positions of nodes that are to be associated
    for ( int i = 1; i <= path.size(); ++i )
    {
        // Calculate index in path
        int index = path.size()-i; // Assumes order in path!!!!!!!

        // Get node index and its parent nodes' indices
        int node_i = path[index];
        int parent1_i = path.parent_tree[index].first;
        int parent2_i = path.parent_tree[index].second;

        if ( parent1_i == -1 || parent2_i == -1 )
        {
            // If root node, position should already be in positions vector, so continue
            continue;
        }

        // Get edge that connects parent nodes
//        int parents_edge_i = (graph.begin() + parent1_i)->edgeByPeer(parent2_i);
        int parents_edge_i = graph.iteratorAtIndex(parent1_i)->edgeByPeer(parent2_i);

        if ( parents_edge_i == -1 )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! No edge connects parents " << parent1_i << " and " << parent2_i << ". This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        // Get triplet that connects parents' edge with new node
//        Graph::const_edge2_iterator parents_edge_it = graph.beginEdges() + parents_edge_i;
        Graph::const_edge2_iterator parents_edge_it = graph.edgeIteratorAtIndex(parents_edge_i);
        if ( parents_edge_it->deleted )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! There's a deleted edge in the path. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        int triplet_i = parents_edge_it->tripletByNode(node_i);
        if ( triplet_i == -1 )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! No triplet connects node " << node_i << " with its parents " << parent1_i << " and " << parent2_i << ". This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        // Parent1 and parent2 are either clockwise or anticlockwise in order with respect to their child node
        // If clockwise (wrong direction) swap parent nodes.
//        Graph::const_edge3_iterator trip_it = graph.beginTriplets() + triplet_i;
        Graph::const_edge3_iterator trip_it = graph.tripletIteratorAtIndex(triplet_i);
        if ( trip_it->deleted )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! There's a deleted triplet in the path. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        if ( parent1_i == trip_it->B && parent2_i == trip_it->A ||
             parent1_i == trip_it->A && parent2_i == trip_it->C ||
             parent1_i == trip_it->C && parent2_i == trip_it->B )
        {
            int tmp = parent1_i;
            parent1_i = parent2_i;
            parent2_i = tmp;
        }

//        int edge_1_i = (graph.begin() + parent1_i)->edgeByPeer(node_i);
        int edge_1_i = graph.iteratorAtIndex(parent1_i)->edgeByPeer(node_i);
        if ( edge_1_i == -1 )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! Edge 1 does not exist. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

//        int edge_2_i = (graph.begin() + parent2_i)->edgeByPeer(node_i);
        int edge_2_i = graph.iteratorAtIndex(parent2_i)->edgeByPeer(node_i);
        if ( edge_2_i == -1 )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! Edge 1 does not exist. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

//        Graph::const_edge2_iterator edge_1_it = graph.beginEdges() + edge_1_i;
        Graph::const_edge2_iterator edge_1_it = graph.edgeIteratorAtIndex(edge_1_i);
        if ( edge_1_it->deleted )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! There's a deleted edge in the path. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        Graph::const_edge2_iterator edge_2_it = graph.edgeIteratorAtIndex(edge_2_i);
        if ( edge_2_it->deleted )
        {
            std::cout << "\033[31m" << "[calculatePositions] ERROR! Bug! There's a deleted edge in the path. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        double l1 = edge_1_it->l;
        double l2 = edge_2_it->l;
        double l3 = parents_edge_it->l;

        // I'm gonna need the squares of two of those lengths
        double l1_sq = l1*l1;
        double l2_sq = l2*l2;
        double l3_sq = l3*l3;

        double s = (l1_sq - l2_sq + l3_sq)/(2*l3);
        double k = sqrt(l1_sq - s*s);

        // Define the triangle frame and espress the position of the new node in the sensor frame
        geo::Vec3d parent_1_pos = positions.measurement.points[positions.node_indices[parent1_i]];
        geo::Vec3d parent_2_pos = positions.measurement.points[positions.node_indices[parent2_i]];
        geo::Vec3d base_x = (parent_2_pos - parent_1_pos)/l3;
        geo::Vec3d base_y = geo::Mat3d(0,-1,0,1,0,0,0,0,1) * base_x;

        positions.append(base_x * s + base_y * k + parent_1_pos, 0.0, node_i); // TODO: magic number for position uncertainty. Maybe use the position uncertainty from path in this?
    }
}

// -----------------------------------------------------------------------------------------------

void associate(const Graph &graph,
               const Measurement &measurement,
               AssociatedMeasurement &associations,
               Measurement &unassociated,
               const int goal_node_i,
               tue::Configuration& config)
/**
 * Given a graph, a measurement, previous associations, a goal node and a maximum association
 * distance, associates measured points with graph nodes and stores these in associations.
 * The input argument 'associations' must contain at least two associated measurement points
 * as these are used to predict the positions of the graph nodes. These associations can
 * either be given as an initial pose or they can be the previous associated measurement
 * resulting from this function, possibly updated using odometry information. Gives back
 * unassociated nodes in the 'unassociated' argument. The goal node must be the node index of
 * the node that is to be found, but may also be -1 if all nodes need to be associated (for
 * example in case of creating a new graph).
 */
{
    Path path;
    associate(graph, measurement, associations, unassociated, goal_node_i, path, config);
}

// -----------------------------------------------------------------------------------------------

void associate(const Graph &graph,
               const Measurement &measurement,
               AssociatedMeasurement &associations,
               Measurement &unassociated,
               const int goal_node_i,
               Path& path,
               tue::Configuration& config)
/**
 * Given a graph, a measurement, previous associations, a goal node and a configuration,
 * associates measured points with graph nodes and stores these in associations. The input
 * argument 'associations' must contain at least two associated measurement points as these
 * are used to predict the positions of the graph nodes, otherwise the function will not give
 * back any associations. These associations can either be given as an initial pose or they
 * can be the previous associated measurement resulting from this function, possibly updated
 * using odometry information. Gives back unassociated nodes in the 'unassociated' argument.
 * The goal node must be the node index of the node that is to be found, but may also be -1
 * if all nodes need to be associated (for example in case of creating a new graph).
 * Additionally gives back the best path that was found to reach the goal node.
 */
{
    // If no points to associate, just return
    if ( measurement.points.size() == 0 )
        return;

    Associator associator;
    associator.configure(config);

    associator.setGraph(graph);

    associator.getAssociations(measurement, associations, goal_node_i);

    associator.getPath(path);
    associator.getUnassociatedPoints(unassociated);
}

// -----------------------------------------------------------------------------------------------

void updateGraph(Graph &graph, const AssociatedMeasurement &associations, bool update_lengths)
/**
 * Function to update an existing graph using an associated measurement.
 * Updates the edge lengths and triplet orders. Does not add measured
 * points to the existing graph.
 */
{
    int i = 0;
    for ( std::vector<int>::const_iterator it_1 = associations.nodes.begin(); it_1 != associations.nodes.end(); ++it_1 )
    {
        int n1 = *it_1;

//        Graph::const_iterator node1_it = graph.begin() + *it_1;
        Graph::const_iterator node1_it = graph.iteratorAtIndex(*it_1);
        if ( node1_it->deleted )
        {
            std::cout << "\033[31m" << "[updateGraph] ERROR! Bug! There's a deleted node in the associations. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        int j = 0;
        for ( std::vector<int>::const_iterator it_2 = associations.nodes.begin(); it_2 != it_1; ++it_2 )
        {
            int n2 = *it_2;
            int e = node1_it->edgeByPeer(n2);

            // Calculate vector between measured points
            geo::Vec3d pt1 = associations.measurement.points[i];
            geo::Vec3d pt2 = associations.measurement.points[j];
            geo::Vec3d d21 = pt2 - pt1;
            double length = d21.length();

            if ( length == 0 )
                std::cout << "\033[31m" << "[GRAPH] updateGraph: ERROR Edge length is zero" << "\033[0m" << std::endl;

            // If edge exists...
            if ( e > -1 )
            {
                // And lengths should be updated...
                if ( update_lengths )
                {
                    // update edge;
                    // TODO: What if triangle inequality doesn't hold anymore? Don't update? Use uncertainty in edge to describe the tension in the triangle?
                    graph.setEdgeLength(e, length);
                }
            }

            // and if it didn't
            else
            {
                // add it to the graph
                // TODO: Magic number! Make default standard deviation configurable
                double default_std_dev = 0.2;
                e = graph.addEdge2(n1, n2, length, default_std_dev);
            }

            Graph::const_edge2_iterator edge_1_it = graph.edgeIteratorAtIndex(e);
            if ( edge_1_it->deleted )
            {
                std::cout << "\033[31m" << "[updateGraph] ERROR! Bug! There's a deleted edge stored in the connected nodes. This is never supposed to happen!" << "\033[0m" << std::endl;
                return;
            }

            int k = 0;
            for ( std::vector<int>::const_iterator it_3 = associations.nodes.begin(); it_3 != it_2; ++it_3 )
            {
                int n3 = *it_3;
                int t = edge_1_it->tripletByNode(n3);

                // Get the third measurement point
                geo::Vec3d pt3 = associations.measurement.points[k];

                // Calculate the vector from node 1 to node 3 (the one from 1 to 2 was already calculated in the enclosing loop)
                geo::Vec3d d31 = pt3 - pt1;

                // Calculate if the angle between the two is positive (counter-clockwise) or negative
                double sign = d21.cross(d31).z;

                // If triplet exists...
                if ( t != -1 )
                {
                    if ( update_lengths )
                    {
                        // get a copy of the triplet.
                        Graph::const_edge3_iterator trip_it = graph.tripletIteratorAtIndex(t);
                        if ( trip_it->deleted )
                        {
                            std::cout << "\033[31m" << "[updateGraph] ERROR! Bug! There's a deleted edge in an edge's list of triplets. This is never supposed to happen!" << "\033[0m" << std::endl;
                            return;
                        }

                        // If order of nodes n1, n2 and n2 is the same as in triplet...
                        if ( n1 == trip_it->A && n2 == trip_it->B && n3 == trip_it->C ||
                             n1 == trip_it->B && n2 == trip_it->C && n3 == trip_it->A ||
                             n1 == trip_it->C && n2 == trip_it->A && n3 == trip_it->B ) // TODO: make this nicer?
                        {
                            // check if that order is clockwise, and if it is...
                            if ( sign < 0 )
                            {
                                // flip it.
                                graph.flipTriplet(t);
                            }
                        }

                        // If it is not the same order, it is the only other...
                        else
                        {
                            // check if that order is clockwise, and if it is...
                            if ( sign > 0 )
                            {
                                // flip it.
                                graph.flipTriplet(t);
                            }
                        }
                    }
                }

                // If the triplet did not exist yet...
                else
                {
                    // add it to the graph in the right order
                    if ( sign > 0 ) // Counter clockwise
                        graph.addEdge3(n1,n2,n3);
                    else
                        graph.addEdge3(n1,n3,n2);
                }
                ++k;
            }
            ++j;
        }
        ++i;
    }
}

// -----------------------------------------------------------------------------------------------

void extendGraph(Graph &graph, const Measurement &unassociated, AssociatedMeasurement &associations, const double edge_std_dev)
/**
 * Function to extend an existing graph using a measurement of
 * unassociated points and the associated measurement from the same
 * measurement. There should not be any overlap between the unassociated
 * and the associated points. Only adds the unassociated points, does
 * not update the existing relations in the graph.
 * TODO: make sure edges are also correct if no update was done (use graph point of associated point, and measurement of unassociated point)
 */
{
    /* If there are not enough points associated with the graph to
     * express positions of new points with respect to them, just
     * return; unless of course the graph does not have enough nodes
     * yet!
     */
    if ( graph.size() > 2 && associations.nodes.size() < 2 )
        return;

    // Add unassociated nodes
    int i = 0;
    for ( std::vector<geo::Vec3d>::const_iterator it = unassociated.points.begin(); it != unassociated.points.end(); ++it )
    {
        int n1 = graph.addNode(Node::generateId());
        geo::Vec3d pt1 = *it;

        // Add edges and triplets between all points in associations
        int j = 0;
        for ( std::vector<int>::const_iterator it_2 = associations.nodes.begin(); it_2 != associations.nodes.end(); ++it_2 )
        {
            int n2 = *it_2;
            geo::Vec3d pt2 = associations.measurement.points[j];

            if ( pt2 == pt1 )
                std::cout << "\033[31m" << "[GRAPH] extendGraph: WARNING Found the same point in associations " <<
                             "and unassociated points vectors. Extendgraph cannot handle this. Did you forget " <<
                             "to associate?" << "\033[0m" << std::endl;

            // Calculate vector between current new point and current associated point
            geo::Vec3d d21 = pt2 - pt1;

            graph.addEdge2(n1, n2, d21.length(), edge_std_dev );

            int k = 0;
            for ( std::vector<int>::const_iterator it_3 = associations.nodes.begin(); it_3 != it_2; ++it_3 )
            {
                int n3 = *it_3;
                geo::Vec3d pt3 = associations.measurement.points[k];

                // If edge between the two associated nodes does not yet exist, add it.
//                if ( graph.getNodes()[n2].edgeByPeer(n3) == -1 )
//                {
//                    std::cout << "Edge between " << n2 << " and " << n3 << " did not exist yet. Adding it..." << std::endl;
//                    geo::Vec3d d23 = pt3 - pt2;
//                    std::cout << "result: " << graph.addEdge2(n2, n3, d23.length()) << std::endl;
//                }

                // Calculate the vectors from node 1 to the other two
                geo::Vec3d d31 = pt3 - pt1;

                // Calculate if the angle between the two is positive (counter-clockwise) or negative
                double sign = d21.cross(d31).z;

                // add it to the graph in the correct order
                if ( sign > 0 )
                    graph.addEdge3(n1,n2,n3);
                else
                    graph.addEdge3(n1,n3,n2);

                ++k;
            }
            ++j;
        }

        // Add newly found point to associations
        associations.append(pt1, unassociated.uncertainties[i], n1);

        ++i;
    }
}

// -----------------------------------------------------------------------------------------------

// TODO: Implement caching pathfinder and positions data so that path and graph node positions are not calculated twice.
AssociatedMeasurement generateVisualization(const Graph& graph, const AssociatedMeasurement& associations, Path& path)
/**
 * Generates an associated measurement (list of points with corresponding node names)
 * given a graph and at least two given associations, which can be used to visualize
 * the graph using the Visualizer class. Uses PathFinder with goal node -1 and the
 * calculatePositions function, so if combined with association, these operations are
 * performed twice, so minimize use of this.
 */
{
    AssociatedMeasurement vis_measurement = associations;

    if ( path.size() == 0 )
    {
        PathFinder pathFinder(graph, vis_measurement.nodes);
        pathFinder.findPath(-1,path);
    }

    calculatePositions(graph, path, vis_measurement);

    for ( unsigned int i = 0; i < path.size(); ++i )
    {
        int node_i = path[i];
        int parent1_i = path.parent_tree[i].first;
        int parent2_i = path.parent_tree[i].second;

        if ( parent1_i > -1 && parent2_i > -1 )
        {
            vis_measurement.measurement.line_list.push_back(vis_measurement.measurement.points[vis_measurement.node_indices[node_i]]);
            vis_measurement.measurement.line_list.push_back(vis_measurement.measurement.points[vis_measurement.node_indices[parent1_i]]);
            vis_measurement.measurement.line_list.push_back(vis_measurement.measurement.points[vis_measurement.node_indices[node_i]]);
            vis_measurement.measurement.line_list.push_back(vis_measurement.measurement.points[vis_measurement.node_indices[parent2_i]]);
        }
    }

    return vis_measurement;
}

// -----------------------------------------------------------------------------------------------

void save(const Graph &graph, const std::string &filename)
/**
 * Stores the given graph in a yaml-file with name 'filename'.
 */
{
    // Instantiate a new config and write graph configuration to it
    tue::Configuration config;
    config.writeArray("nodes");
    std::string s("'");
    for ( Graph::const_iterator it = graph.begin(); it != graph.end(); ++it )
    {
        config.addArrayItem();
        config.setValue("id",s+ it->id +s);
        config.endArrayItem();
    }
    config.endArray();

    config.writeArray("edges");
    for ( Graph::const_edge2_iterator it = graph.beginEdges(); it != graph.endEdges(); ++it )
    {
        config.addArrayItem();
//        config.setValue("n1",s+(graph.begin() + it->A)->id+s);
//        config.setValue("n2",s+(graph.begin() + it->B)->id+s);
        config.setValue("n1",s+(graph.iteratorAtIndex(it->A))->id+s);
        config.setValue("n2",s+(graph.iteratorAtIndex(it->B))->id+s);
        config.setValue("length",it->l);
        config.endArrayItem();
    }
    config.endArray();

    config.writeArray("triplets");
    for ( Graph::const_edge3_iterator it = graph.beginTriplets(); it != graph.endTriplets(); ++it )
    {
        config.addArrayItem();
//        config.setValue("n1",s+(graph.begin() + it->A)->id+s);
//        config.setValue("n2",s+(graph.begin() + it->B)->id+s);
//        config.setValue("n3",s+(graph.begin() + it->C)->id+s);
        config.setValue("n1",s+(graph.iteratorAtIndex(it->A))->id+s);
        config.setValue("n2",s+(graph.iteratorAtIndex(it->B))->id+s);
        config.setValue("n3",s+(graph.iteratorAtIndex(it->C))->id+s);
        config.endArrayItem();
    }
    config.endArray();

    std::cout << "Writing " << graph.size() << " nodes, " << graph.numEdges() << " edges and " << graph.numTriplets() << " triplets to " << filename.c_str() << std::endl;

    // Convert config to yaml string and write to file.
    std::ofstream file;
    file.open (filename.c_str());
    file << config.toYAMLString();
    file.close();
}

}
