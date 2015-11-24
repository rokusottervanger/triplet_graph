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

namespace triplet_graph
{

// -----------------------------------------------------------------------------------------------

int findNodeByID(const Graph& g, const std::string &id)
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

int getThirdNode(const Edge3& triplet, const int node1, const int node2)
{
    if ( triplet.A == node1 && triplet.B == node2 )
        return triplet.C;
    else if ( triplet.B == node1 && triplet.C == node2 )
        return triplet.A;
    else if ( triplet.C == node1 && triplet.A == node2 )
        return triplet.B;
    else if ( triplet.A == node2 && triplet.B == node1 )
        return triplet.C;
    else if ( triplet.B == node2 && triplet.C == node1 )
        return triplet.A;
    else if ( triplet.C == node2 && triplet.A == node1 )
        return triplet.B;
    else
        return -1;
}

// -----------------------------------------------------------------------------------------------

double findPath(const Graph& graph, const std::vector<int>& source_nodes, const int target_node, Path& path)
{
    typedef std::pair< double, int > CostInt; // First is a cost, second is an edge or a node index
    // In the initial graph search: first is the sum of the costs (so far) to get to the nodes connected by an edge, second is the respective edge index
    // In the path trace search:

    const double inf = 1e38;

    // get a copy of the nodes, edges and triplets in the graph
    std::vector<Node>  nodes    = graph.getNodes();
    std::vector<Edge2> edges    = graph.getEdge2s();
    std::vector<Edge3> triplets = graph.getEdge3s();

    // Track visited edges and triplets and cost to nodes
    std::vector<double> ns(graph.size(),inf);
    std::vector<double> es(edges.size(),inf);
    std::vector<double> ts(triplets.size(),inf);

    /* The priority queue holds couples of nodes (edges) to be handled, sorted by
     * the sum of the cost to get to those nodes. The cost to get to the first
     * pair of nodes is obviously zero.
     */
    std::priority_queue<CostInt, std::vector<CostInt>, std::greater<CostInt> > Q;

    // Find all edges connecting the source nodes and add those edges to Q
    for ( std::vector<int>::const_iterator it_1 = source_nodes.begin(); it_1 != source_nodes.end(); ++it_1 )
    {
        if ( *it_1 == -1 )
            std::cout << "[FIND_PATH] Warning! Input node index is -1!" << std::endl;
        else
        {
            for ( std::vector<int>::const_iterator it_2 = source_nodes.begin(); it_2 != it_1; ++it_2 )
            {
                if ( *it_2 == -1 )
                    std::cout << "[FIND_PATH] Warning! Input node index is -1!" << std::endl;
                else
                {
                    int edge = nodes[*it_1].edgeByPeer(*it_2);
//                    int edge = getConnectingEdge2(graph,*it_1,*it_2);
                    if ( edge != -1 )
                    {
                        Q.push(CostInt(0,edge));
                        es[edge] = 0;
                    }
                }
            }
            ns[*it_1] = 0;
        }
    }

    /* The path is to contain the series of nodes to get from the source nodes to
     * the target node. To construct this path, the prevs vector is maintained,
     * holding for every visited node the previous node and -1 for ever non-
     * visited node.
     */
    std::vector<int> prevs(nodes.size(),-1);


    while(!Q.empty())
    {
        // Take the cheapest edge (cost is sum of node costs so far) from the queue
        int u = Q.top().second; // current edge, cheapest pair of nodes so far
        Q.pop();

        // If pair of nodes already visited, continue
        if ( es[u] == -1 )
            continue;

        // When the target is reached in current edge's A or B node, trace back path
        if ( edges[u].A == target_node || edges[u].B == target_node)
        {
            std::priority_queue<CostInt, std::vector<CostInt>, std::less<CostInt> > trace;

            // Push target node into trace
            trace.push(CostInt(ns[target_node],target_node));
            int n, e;

            while ( !trace.empty() )
            {
                n = trace.top().second;
                trace.pop();

                std::cout << "n = " << n << std::endl;

                // The current node refers to its originating edge using prevs
                e = prevs[n];

                // If edge not yet visited and pushed to path, push both node A and node B of this edge to trace and push current node to path
                if ( e != -1 )
                {
                    // Don't add source nodes (cost == 0) to trace
                    int na = edges[e].A;
                    if ( ns[na] != 0 )
                        trace.push(CostInt(ns[na],na));

                    int nb = edges[e].B;
                    if ( ns[nb] != 0 )
                        trace.push(CostInt(ns[nb],nb));

                    path.push_back(n);
                    path.parent_tree.push_back(std::make_pair(na,nb));

                    prevs[n] = -1;
                }
            }

            // Finally, add source nodes to path
            path.insert(path.end(), source_nodes.begin(), source_nodes.end());
            path.parent_tree.resize(path.size(),std::make_pair(-1,-1));

            // When finished, return cost to target node
            return ns[target_node];
        }

        std::vector<int> common_triplets = nodes[edges[u].A].tripletsByPeer(edges[u].B);
//        std::vector<int> common_triplets = getCommonTriplets(graph,edges[u].A,edges[u].B);

        // Run through common triplets of the current pair of nodes
        for ( std::vector<int>::iterator t_it = common_triplets.begin(); t_it != common_triplets.end(); ++t_it )
        {
            // If this triplet was already visited, continue
            if ( ts[*t_it] == -1 )
                continue;
            ts[*t_it] = -1; // TODO: Is this OK?

            // Retrieve the right node from the triplet.
            int v = getThirdNode(triplets[*t_it],edges[u].A,edges[u].B);

            // TODO: Calculate weight using the two edges connecting the third node to the two base nodes
            double w = 1.0;

            // If path to third node is cheaper than before, update cost to that node, add the cheapest connecting edge to priority queue
            // of potential nodes to visit and record what the previous node was.
            double new_cost = ns[edges[u].A] + ns[edges[u].B] + w; // TODO: Now taking sum of node costs plus new cost, is this what I want?
            if (ns[v] > new_cost)
            {
                ns[v] = new_cost;

                // Loop through all neighbors of current node (v) and add connecting edges to queue if neighbor is visited
                for ( std::vector<int>::iterator e_it = nodes[v].edges.begin(); e_it !=nodes[v].edges.end(); ++e_it )
                {
//                    int neighbor = getSecondNode(edges[*e_it],v);
//                    int neighbor = nodes[v].peerByEdge(*e_it);
                    int neighbor = edges[*e_it].getOtherNode(v);

                    if ( ns[neighbor] < inf )
                        Q.push(CostInt(new_cost, *e_it));
                }

                // Store edge that lead to this node
                prevs[v] = u;
            }
        }

        // After visiting edge, mark it visited using vector of edge weights
        es[u] = -1;
    }
}

// -----------------------------------------------------------------------------------------------

bool configure(Graph& g, tue::Configuration &config)
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

void associate(Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations, const geo::Transform3d &delta, const int goal_node_i)
{
    double max_distance = 0.1; // TODO: magic number, parameterize!
    double max_distance_sq = max_distance*max_distance;

    std::vector<Edge2> edges = graph.getEdge2s();
    std::vector<Edge3> triplets = graph.getEdge3s();

    std::cout << 1 << std::endl;

    Path path;
    if ( associations.nodes.size() > 1 )
    {
        if ( goal_node_i == -1 )
        {
            for ( Graph::const_iterator it = graph.begin(); it != graph.end(); ++it )
            {
                path.push_back(it - graph.begin());

            }
            path.parent_tree.push_back(std::make_pair(-1,-1));
            path.parent_tree.push_back(std::make_pair(-1,-1));
            path.parent_tree.push_back(std::make_pair(0,1));
        }
        else
        {
            findPath(graph,associations.nodes,goal_node_i,path);
        }
    }
    else
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! Not enough initial associations given" << "\033[0m" << std::endl;
        return;
    }
    std::cout << "Nodes to be associated (path): " << path << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate positions of nodes on path in sensor frame
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Vec3d> positions(graph.size());

    // Add prior associations to positions vector to
    for ( int i = 0; i < associations.nodes.size(); ++i )
        positions[associations.nodes[i]] = delta.inverse() * associations.measurement.points[i];
    associations.measurement.points.clear();
    associations.nodes.clear();

    for ( int i = 0; i < positions.size(); ++i )
    {
        associations.nodes.push_back(i);
        associations.measurement.points.push_back(positions[i]);
    }

    std::cout << 3 << std::endl;

    // TODO: Split up position calculation of nodes to use also for visualisation of graph?
    for ( int i = 1; i <= path.size(); ++i )
    {
        std::cout << "path.size() " << path.size() << std::endl;
        // Calculate index in path
        int index = path.size()-i;

        std::cout << "index " << index << std::endl;

        // Get node index and its parent nodes' indices
        int node_i = path[index];
        int parent1_i = path.parent_tree[node_i].first;
        int parent2_i = path.parent_tree[node_i].second;

        if ( parent1_i == -1 || parent2_i == -1 )
        {
            if ( parent1_i != -1 || parent2_i != -1 )
            {
                std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! Node " << node_i << " has one parent. This is never supposed to happen!" << "\033[0m" << std::endl;
            }
            continue;
        }

        std::cout << 5 << std::endl;

        // Get edge that connects parent nodes
        int parents_edge_i = (graph.begin() + parent1_i)->edgeByPeer(parent2_i);
        if ( parents_edge_i == -1 )
        {
            std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! No edge connects the parents. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        // Get triplet that connects parents' edge with new node
        int triplet_i = edges[parents_edge_i].tripletByNode(node_i);
        if ( triplet_i == -1 )
        {
            std::cout << "\033[31m" << "[GRAPH] ERROR! Bug! No triplet connects node with its parents. This is never supposed to happen!" << "\033[0m" << std::endl;
            return;
        }

        std::cout << 6 << std::endl;

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

        std::cout << 7 << std::endl;

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

        std::cout << 8 << std::endl;

        Edge2 edge_1 = edges[edge_1_i];
        Edge2 edge_2 = edges[edge_2_i];
        Edge2 parents_edge = edges[parents_edge_i];

        std::cout << 9 << std::endl;

        // In notes, names of l1 and l2 were swapped, so:
        double l1 = edge_2.l;
        double l2 = edge_1.l;
        double l3 = parents_edge.l;

        // I'm gonna need the squares of those lengths
        double l1_sq = l1*l1;
        double l2_sq = l2*l2;
        double l3_sq = l3*l3;

        // Now calculate k and s, which are the coordinates of the new node in the triangle frame
        double k_sq = ( l2_sq + l1_sq - l3_sq )/2.0;
        double k = sqrt(k_sq);
        double s = sqrt(l2_sq - k_sq);

        std::cout << 10 << std::endl;

        // Define the triangle frame and espress the position of the new node in the sensor frame
        geo::Vec3d base_x = (positions[parent2_i] - positions[parent1_i])/edges[parents_edge_i].l;
        geo::Vec3d base_y = geo::Mat3d(0,-1,0,1,0,0,0,0,1) * base_x;
        positions[node_i] = base_x * s + base_y * k + positions[parent1_i];

        std::cout << 11 << std::endl;

        // For this graph node, go through the points in the measurement and associate the closest point within a bound with this node.
        // TODO: match a node to each measurement point instead of trying to match a measurement point to each node?
        // TODO: make sure that a measured point is never associated with two different nodes
        // TODO: first try to associate farthest point in path, if that doesn't work, proceed with closer points.
        double best_dist_sq = 1.0e9;
        geo::Vec3d best_guess;
        for ( std::vector<geo::Vec3d>::const_iterator it = measurement.points.begin(); it != measurement.points.end(); ++it )
        {
            double dx_sq = (positions[node_i] - *it).length2();
            if ( dx_sq < max_distance_sq )
            {
                if ( dx_sq < best_dist_sq )
                {
                    best_dist_sq = dx_sq;
                    best_guess = *it;
                }
            }
        }

        std::cout << 12 << std::endl;

        if ( best_dist_sq < 1.0e9 )
        {
            associations.nodes.push_back(node_i);
            associations.measurement.points.push_back(best_guess);
            std::cout << 13 << std::endl;
            // TODO: make sure that one measurement without (associated) points does not let the robot get lost
        }
        std::cout << 14 << std::endl;
    }
}

// -----------------------------------------------------------------------------------------------

void updateGraph(Graph &graph, const AssociatedMeasurement &associations)
/* Function to update an existing graph using an associated measurement.
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
        Node node1 = *(graph.begin() + n1);

        int j = i+1;
        for ( std::vector<int>::const_iterator it_2 = it_1+1; it_2 != associations.nodes.end(); ++it_2 )
        {
            int n2 = *it_2;
            int e = node1.edgeByPeer(n2);

            // Calculate vector between measured points
            geo::Vec3d pt1 = associations.measurement.points[i];
            geo::Vec3d pt2 = associations.measurement.points[j];
            geo::Vec3d d21 = pt2 - pt1;

            // If edge exists...
            if ( e > -1 )
            {
                // update edge;
                graph.setEdgeLength(e, d21.length());
            }

            // and if it didn't
            else
            {
                // add it to the graph
                e = graph.addEdge2(n1, n2, d21.length() );
            }

            Edge2 edge1 = edges[e];

            int k = j+1;
            for ( std::vector<int>::const_iterator it_3 = it_2+1; it_3 != associations.nodes.end(); ++it_3 )
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
                if ( t > -1 )
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

                // If the triplet did not exist yet...
                else
                {
                    // add it to the graph
                    graph.addEdge3(n1,n2,n3);
                }
                ++k;
            }
            ++j;
        }
        ++i;
    }
}

void extendGraph(Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations)
/* Function to extend an existing graph using a measurement and the
 * associated measurement derived from that. The associated measurement
 * must be a subset of the measurement. Only adds the unassociated points,
 * does not update the existing relations in the graph.
 */
{
    // Add unassociated nodes
    int i = 0; // Index in vector of associations
    for ( std::vector<geo::Vec3d>::const_iterator it = measurement.points.begin(); it != measurement.points.end(); ++it )
    {
        // If point is already associated, continue. (Assumes that order of points in associated measurement is equal to order in measurement)
        if ( i < associations.measurement.points.size() && *it == associations.measurement.points[i] ) // TODO: Make sure i does not run out of this vector?
        {
            ++i;
            continue;
        }

        int n1 = graph.addNode(Node::generateId());
        geo::Vec3d pt1 = *it;

        // Add edges and triplets between all other points in the measurement
        int j = 0;
        for ( std::vector<int>::const_iterator it_2 = associations.nodes.begin(); it_2 != associations.nodes.end(); ++it_2 )
        {
            int n2 = *it_2;
            geo::Vec3d pt2 = associations.measurement.points[j];

            // Calculate vector between current new point and current associated point
            geo::Vec3d d21 = pt2 - pt1;

            // Add edge to graph with length of diff vector
            graph.addEdge2(n1, n2, d21.length() );

            int k = j+1;
            for ( std::vector<int>::const_iterator it_3 = it_2+1; it_3 != associations.nodes.end(); ++it_3 )
            {
                int n3 = *it_3;
                geo::Vec3d pt3 = measurement.points[k];

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

        ++i;
    }
}

void save(const Graph &graph, const std::string &filename)
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

    std::cout << "Writing " << graph.size() << " nodes, " << edges.size() << " edges and " << triplets.size() << " to " << filename << std::cout;

    // Convert config to yaml string and write to file.
    std::ofstream file;
    file.open (filename.c_str());
    file << config.toYAMLString();
    file.close();

    std::cout << "Done!" << std::endl;
}

}
