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
            return (it - g.begin());
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
    if (config.readArray("nodes"))
    {
        while (config.nextArrayItem())
        {
            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled;
            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
                continue;

            std::string id;
            if (!config.value("id", id))
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

    if (config.readArray("edges"))
    {
        while(config.nextArrayItem())
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
                if (config.value("length", length, tue::REQUIRED))
                {
                    g.addEdge2(n1,n2,length);
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

    if (config.readArray("triplets"))
    {
        while(config.nextArrayItem())
        {
            int n1, n2, n3;
            std::string id1, id2, id3;
            if ( config.value("n1", id1) && config.value("n2", id2) && config.value("n3", id3))
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

// TODO: make more efficient! Calculates positions from scratch every iteration,
// while it would be much more efficient to cache those and transform them to the
// pose of the new measurement
void calculatePositions(const Graph &graph, std::vector<geo::Vec3d>& positions, const Path& path)
/**
 * Given a graph, a path and a sparse vector of positions, calculates the positions of
 * all nodes in the path. The vector of positions must be of the same size as the graph
 * and must contain at least the positions of the root nodes of the given path.
 */
{
    std::vector<Edge2> edges = graph.getEdge2s();
    std::vector<Edge3> triplets = graph.getEdge3s();

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
        int parents_edge_i = (graph.begin() + parent1_i)->edgeByPeer(parent2_i);
        if ( parents_edge_i == -1 )
        {
            std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! No edge connects parents " << parent1_i << " and " << parent2_i << ". This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        // Get triplet that connects parents' edge with new node
        int triplet_i = edges[parents_edge_i].tripletByNode(node_i);
        if ( triplet_i == -1 )
        {
            std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! No triplet connects node " << node_i << " with its parents " << parent1_i << " and " << parent2_i << ". This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        // Parent1 and parent2 are either clockwise or anticlockwise in order with respect to their child node
        // If clockwise (wrong direction) swap parent nodes.
        Edge3 trip = triplets[triplet_i];

        if ( parent1_i == trip.B && parent2_i == trip.A ||
             parent1_i == trip.A && parent2_i == trip.C ||
             parent1_i == trip.C && parent2_i == trip.B ) // TODO: make this nicer?
        {
            int tmp = parent1_i;
            parent1_i = parent2_i;
            parent2_i = tmp;
        }

        int edge_1_i = (graph.begin() + parent1_i)->edgeByPeer(node_i);
        if ( edge_1_i == -1 )
        {
            std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! Edge 1 does not exist. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        int edge_2_i = (graph.begin() + parent2_i)->edgeByPeer(node_i);
        if ( edge_2_i == -1 )
        {
            std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! Edge 1 does not exist. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        Edge2 edge_1 = edges[edge_1_i];
        Edge2 edge_2 = edges[edge_2_i];
        Edge2 parents_edge = edges[parents_edge_i];

        double l1 = edge_1.l;
        double l2 = edge_2.l;
        double l3 = parents_edge.l;

        // I'm gonna need the squares of two of those lengths
        double l1_sq = l1*l1;
        double l2_sq = l2*l2;
        double l3_sq = l3*l3;

        double s = (l1_sq - l2_sq + l3_sq)/(2*l3);
        double k = sqrt(l1_sq - s*s);

        // Define the triangle frame and espress the position of the new node in the sensor frame
        geo::Vec3d base_x = (positions[parent2_i] - positions[parent1_i])/edges[parents_edge_i].l;
        geo::Vec3d base_y = geo::Mat3d(0,-1,0,1,0,0,0,0,1) * base_x;

        positions[node_i] = base_x * s + base_y * k + positions[parent1_i];
    }
}

// -----------------------------------------------------------------------------------------------

void associate(Graph &graph,
               const Measurement &measurement,
               AssociatedMeasurement &associations,
               Measurement &unassociated,
               const int goal_node_i,
               const double max_distance)
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
    associate(graph, measurement, associations, unassociated, goal_node_i, path, max_distance);
}

// -----------------------------------------------------------------------------------------------

void associate(Graph &graph,
               const Measurement &measurement,
               AssociatedMeasurement &associations,
               Measurement &unassociated,
               const int goal_node_i,
               Path& path,
               const double max_distance)
/**
 * Given a graph, a measurement, previous associations, a goal node and a maximum association
 * distance, associates measured points with graph nodes and stores these in associations.
 * The input argument 'associations' must contain at least two associated measurement points
 * as these are used to predict the positions of the graph nodes. These associations can
 * either be given as an initial pose or they can be the previous associated measurement
 * resulting from this function, possibly updated using odometry information. Gives back
 * unassociated nodes in the 'unassociated' argument. The goal node must be the node index of
 * the node that is to be found, but may also be -1 if all nodes need to be associated (for
 * example in case of creating a new graph). Additionally gives back the best path that was
 * found to reach the goal node.
 */
{

    // If no points to associate, just return
    if (measurement.points.size() == 0 )
        return;

    double max_distance_sq = max_distance*max_distance;

    // Now find a path through the graph to the goal
    PathFinder pathFinder(graph, associations.nodes);
    pathFinder.findPath(goal_node_i, path);

    std::cout << "Found path: " << path << std::endl;

    bool local_nn = false;

    if ( local_nn )
    {

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // -     Calculate positions of nodes on path in sensor frame
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        std::vector<geo::Vec3d> positions(graph.size());

        // Add prior associations to positions vector to
        for ( int i = 0; i < associations.nodes.size(); ++i )
        {
            positions[associations.nodes[i]] = associations.measurement.points[i];
        }
        associations.measurement.points.clear();
        associations.nodes.clear();

        calculatePositions(graph,positions,path);


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // -     Nearest neighbor association
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        // TODO: Check if association goes well (in case of false positives, false negatives, etc.)
        for ( std::vector<geo::Vec3d>::const_iterator it_m = measurement.points.begin(); it_m != measurement.points.end(); ++it_m )
        {
            double best_dist_sq = 1.0e9;
            int best_guess = -1;
            geo::Vec3d best_pos;

            // Go through nodes in path (nodes to be associated from far to close) to check if one associates with the measured point
            for ( Path::iterator it_p = path.begin(); it_p != path.end(); ++it_p )
            {
                int i = *it_p;
                double dx_sq = (*it_m - positions[i]).length2();
                if ( dx_sq < max_distance_sq )
                {
                    if ( dx_sq < best_dist_sq )
                    {
                        best_dist_sq = dx_sq;
                        best_pos = *it_m;
                        best_guess = i;
                    }
                }
            }

            // Check if an association is made, and if so, push it into associations
            if ( best_guess > -1 )
            {
                associations.nodes.push_back(best_guess);
                associations.measurement.points.push_back(best_pos);
            }
            else
            {
                unassociated.points.push_back(*it_m);
            }
        }
    }
    else
    {
        Associator associator;
        associator.configure();

        associator.setGraph(graph);
        associator.setAssociations(associations);

        associations.nodes.clear();
        associations.measurement.points.clear();

        associator.getAssociations(measurement, associations);
        associator.getPath(path);
        associator.getUnassociatedPoints(unassociated);
    }
}

// -----------------------------------------------------------------------------------------------

void updateGraph(Graph &graph, const AssociatedMeasurement &associations, bool update_lengths)
/**
 * Function to update an existing graph using an associated measurement.
 * Updates the edge lengths and triplet orders. Does not add measured
 * points to the existing graph.
 */
{
    std::vector<Edge2> edges = graph.getEdge2s();
    std::vector<Edge3> triplets = graph.getEdge3s();

    int i = 0;
    for ( std::vector<int>::const_iterator it_1 = associations.nodes.begin(); it_1 != associations.nodes.end(); ++it_1 )
    {
        int n1 = *it_1;
        Node node1 = graph.getNodes()[n1];

        int j = 0;
        for ( std::vector<int>::const_iterator it_2 = associations.nodes.begin(); it_2 != it_1; ++it_2 )
        {
            int n2 = *it_2;
            int e = node1.edgeByPeer(n2);

            // Calculate vector between measured points
            geo::Vec3d pt1 = associations.measurement.points[i];
            geo::Vec3d pt2 = associations.measurement.points[j];
            geo::Vec3d d21 = pt2 - pt1;
            double length = d21.length();

            if (length == 0)
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
                e = graph.addEdge2(n1, n2, length );
            }

            edges = graph.getEdge2s();
            Edge2 edge1 = edges[e];

            int k = 0;
            for ( std::vector<int>::const_iterator it_3 = associations.nodes.begin(); it_3 != it_2; ++it_3 )
            {
                int n3 = *it_3;
                int t = edge1.tripletByNode(n3);

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
                        Edge3 triplet = triplets[t];

                        // If order of nodes n1, n2 and n2 is the same as in triplet...
                        if ( n1 == triplet.A && n2 == triplet.B && n3 == triplet.C ||
                             n1 == triplet.B && n2 == triplet.C && n3 == triplet.A ||
                             n1 == triplet.C && n2 == triplet.A && n3 == triplet.B ) // TODO: make this nicer?
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

void extendGraph(Graph &graph, const Measurement &unassociated, AssociatedMeasurement &associations)
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

            graph.addEdge2(n1, n2, d21.length() );

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
        associations.nodes.push_back(n1);
        associations.measurement.points.push_back(pt1);
    }
}

// -----------------------------------------------------------------------------------------------

// TODO: Implement caching pathfinder and positions data so that path and graph node positions are not calculated twice.
AssociatedMeasurement generateVisualization(const Graph& graph, const AssociatedMeasurement& associations)
/**
 * Generates an associated measurement (list of points with corresponding node names)
 * given a graph and at least two given associations, which can be used to visualize
 * the graph using the Visualizer class. Uses PathFinder with goal node -1 and the
 * calculatePositions function, so if combined with association, these operations are
 * performed twice, so minimize use of this.
 */
{
    std::vector<geo::Vec3d> positions(graph.size());\
    for ( unsigned int i = 0; i < associations.nodes.size(); ++i )
    {
        positions[associations.nodes[i]] = associations.measurement.points[i];
    }

    PathFinder pathFinder(graph, associations.nodes);
    Path path;
    pathFinder.findPath(-1,path);

    calculatePositions(graph, positions, path);

    AssociatedMeasurement vis_measurement;
    vis_measurement.measurement.frame_id = associations.measurement.frame_id;
    vis_measurement.measurement.time_stamp = associations.measurement.time_stamp;

    for ( unsigned int i = 0; i < path.size(); ++i )
    {
        int node_i = path[i];
        int parent1_i = path.parent_tree[i].first;
        int parent2_i = path.parent_tree[i].second;

        vis_measurement.measurement.line_list.push_back(positions[node_i]);
        vis_measurement.measurement.line_list.push_back(positions[parent1_i]);
        vis_measurement.measurement.line_list.push_back(positions[node_i]);
        vis_measurement.measurement.line_list.push_back(positions[parent2_i]);
        vis_measurement.measurement.points.push_back(positions[node_i]);
        vis_measurement.nodes.push_back(node_i);
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

    std::vector<Edge2> edges = graph.getEdge2s();
    config.writeArray("edges");
    for ( std::vector<Edge2>::const_iterator it = edges.begin(); it != edges.end(); ++it )
    {
        config.addArrayItem();
        config.setValue("n1",s+(graph.begin() + it->A)->id+s);
        config.setValue("n2",s+(graph.begin() + it->B)->id+s);
        config.setValue("length",it->l);
        config.endArrayItem();
    }
    config.endArray();

    std::vector<Edge3> triplets = graph.getEdge3s();
    config.writeArray("triplets");
    for ( std::vector<Edge3>::const_iterator it = triplets.begin(); it != triplets.end(); ++it )
    {
        config.addArrayItem();
        config.setValue("n1",s+(graph.begin() + it->A)->id+s);
        config.setValue("n2",s+(graph.begin() + it->B)->id+s);
        config.setValue("n3",s+(graph.begin() + it->C)->id+s);
        config.endArrayItem();
    }
    config.endArray();

    std::cout << "Writing " << graph.size() << " nodes, " << edges.size() << " edges and " << triplets.size() << " triplets to " << filename.c_str() << std::endl;

    // Convert config to yaml string and write to file.
    std::ofstream file;
    file.open (filename.c_str());
    file << config.toYAMLString();
    file.close();
}

}
