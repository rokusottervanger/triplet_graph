#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>

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

int getConnectingEdge2(const Graph& graph, const int Node1, const int Node2)
{
    std::vector<Edge2> edges = graph.getEdge2s();

    Node n1 = *(graph.begin() + Node1);

    // TODO: Make this more efficient by giving nodes a map from node indices to edge indices?
    for( std::vector<int>::iterator it = n1.edges.begin(); it != n1.edges.end(); ++it )
    {
        Edge2 edge = edges[*it];
        if ( edge.A == Node2 || edge.B == Node2 )
            return *it;
    }

    return -1;
}

// -----------------------------------------------------------------------------------------------

int getSecondNode(const Edge2& edge, const int node)
{
    if ( edge.A == node )
        return edge.B;
    else if ( edge.B == node )
        return edge.A;
    else
        return -1;
}

// -----------------------------------------------------------------------------------------------

std::vector<int> getCommonTriplets(const Graph& graph, const int Node1, const int Node2)
{
    std::vector<Edge3> triplets = graph.getEdge3s();

    Node n1 = *(graph.begin() + Node1);

    // TODO: Make this more efficient by giving nodes a map from node indices to triplet indices?
    std::vector<int> common_triplets;

    for( std::vector<int>::iterator it = n1.triplets.begin(); it != n1.triplets.end(); ++it )
    {
        Edge3 triplet = triplets[*it];
        if ( triplet.A == Node2 || triplet.B == Node2 || triplet.C == Node2 )
            common_triplets.push_back(*it);
    }

    return common_triplets;
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
                    int edge = getConnectingEdge2(graph,*it_1,*it_2);
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

        std::vector<int> common_triplets = getCommonTriplets(graph,edges[u].A,edges[u].B);

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
                    int neighbor = getSecondNode(edges[*e_it],v);

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
    if (config.readArray("objects"))
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
        return false;

    if (config.readArray("edges"))
    {
        while(config.nextArrayItem())
        {
            std::string id1, id2;
            if (!config.value("n1", id1) || !config.value("n2", id2))
                continue;

            int n1 = findNodeByID(g,id1);
            int n2 = findNodeByID(g,id2);

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
            std::string id1, id2, id3;
            if (!config.value("n1", id1) || !config.value("n2", id2) || !config.value("n3", id3))
                continue;

            int n1 = findNodeByID(g,id1);
            int n2 = findNodeByID(g,id2);
            int n3 = findNodeByID(g,id3);

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

struct AssociatedMeasurement
{
    // A measurement containing all associated points
    Measurement measurement;

    // A vector of node indices to which the measurement is associated (index of point in measurement is the same as index of node number in nodes)
    std::vector<int> nodes;
};

void associate(Graph &graph, const Measurement &measurement, AssociatedMeasurement &associations, const geo::Pose3D &delta, const int goal_node_i)
{
    double max_distance = 0.1; // TODO: magic number, parameterize!
    double max_distance_sq = max_distance*max_distance;

    std::vector<Edge2> edges = graph.getEdge2s();
    std::vector<Edge3> triplets = graph.getEdge3s();

    Path path;
    if ( associations.nodes.size() > 1 )
        double cost = findPath(graph,associations.nodes,goal_node_i,path);
    else
    {
        std::cout << "\033[31m" << "[GRAPH] ERROR! Not enough initial associations given" << "\033[0m" << std::endl;
        return;
    }

    // TODO: add delta to poses:
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate delta movement based on odom (fetched from TF)
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    if (!tf_listener_->waitForTransform(odom_frame_id_, base_link_frame_id_, laser_msg_->header.stamp, ros::Duration(1.0)))
//    {
//        ROS_WARN_STREAM("[ED LOCALIZATION] Cannot get transform from '" << odom_frame_id_ << "' to '" << base_link_frame_id_ << "'.");
//        return;
//    }

//    geo::Pose3D odom_to_base_link;
//    Transform movement;

//    try
//    {
//        tf::StampedTransform odom_to_base_link_tf;

//        tf_listener_->lookupTransform(odom_frame_id_, base_link_frame_id_, laser_msg_->header.stamp, odom_to_base_link_tf);

//        geo::convert(odom_to_base_link_tf, odom_to_base_link);

//        if (have_previous_pose_)
//        {
//            geo::Pose3D delta = previous_pose_.inverse() * odom_to_base_link;

//            // Convert to 2D transformation
//            geo::Transform2 delta_2d(geo::Mat2(delta.R.xx, delta.R.xy,
//                                               delta.R.yx, delta.R.yy),
//                                     geo::Vec2(delta.t.x, delta.t.y));

//            movement.set(delta_2d);
//        }
//        else
//        {
//            movement.set(geo::Transform2::identity());
//        }

//        previous_pose_ = odom_to_base_link;
//        have_previous_pose_ = true;
//    }
//    catch (tf::TransformException e)
//    {
//        std::cout << "[ED LOCALIZATION] " << e.what() << std::endl;

//        if (!have_previous_pose_)
//            return;

//        odom_to_base_link = previous_pose_;

//        movement.set(geo::Transform2::identity());
//    }


    // Calculate positions of nodes on path in sensor frame
    std::vector<geo::Vec3d> positions(graph.size());

    // Add prior associations to positions vector to
    for ( int i = 0; i < associations.nodes.size(); ++i )
        positions[associations.nodes[i]] = associations.measurement.points[i];

    // TODO: Use position calculation for visualisation of graph?
    for ( int i = 1; i <= path.size(); ++i )
    {
        // Calculate index in path
        int index = path.size()-i;

        // Get node index and its parent nodes' indices
        int node_i = path[index];
        int parent1_i = path.parent_tree[node_i].first;
        int parent2_i = path.parent_tree[node_i].second;

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

        // Parent1 and parent2 are either clockwise or anticlockwise in order with respect to their child node
        // If clockwise (wrong direction) swap parent nodes.
        Edge3 trip = triplets[triplet_i];
        if ( parent1_i == trip.B && parent2_i == trip.A || parent1_i == trip.A && parent2_i == trip.C || parent1_i == trip.C && parent2_i == trip.B )
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

        // Define the triangle frame and espress the position of the new node in the sensor frame
        geo::Vec3d base_x = (positions[parent2_i] - positions[parent1_i])/edges[parents_edge_i].l;
        geo::Vec3d base_y = geo::Mat3d(0,-1,0,1,0,0,0,0,1) * base_x;
        positions[node_i] = base_x * s + base_y * k + positions[parent1_i];

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
        if ( best_dist_sq < 1.0e9 )
        {
            associations.nodes.push_back(node_i);
            associations.measurement.points.push_back(best_guess);
        }

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
        Node node1 = *(graph.begin() + *it_1);

        int j = i+1;
        for ( std::vector<int>::const_iterator it_2 = it_1+1; it_2 != associations.nodes.end(); ++it_2 )
        {
            Node node2 = *(graph.begin() + *it_2);
            int e = node1.edgeByPeer(*it_2);

            // Calculate vector between measured points
            geo::Vec3d diff = associations.measurement.points[i]-associations.measurement.points[j];

            // if edge exists...
            if ( e > -1 )
            {
                // update edge
                graph.setEdgeLength(e, diff.length());
            }
            // if it didn't
            else
            {
                // add it to the graph
                e = graph.addEdge2(*it_1, *it_2, diff.length() );
            }

            Edge2 edge1 = edges[e];

            int k = j+1;
            for ( std::vector<int>::const_iterator it_3 = it_2+1; it_3 != associations.nodes.end(); ++it_3 )
            {
                Node node3 = *(graph.begin() + *it_3);
                int t = edge1.tripletByNode(*it_3);

                // Check what the counter-clockwise order of nodes is
                geo::Vec3d pt1 = associations.measurement.points[i];
                geo::Vec3d pt2 = associations.measurement.points[j];
                geo::Vec3d pt3 = associations.measurement.points[k];

                geo::Vec3d d31 = pt3 - pt1;
                geo::Vec3d d21 = pt2 - pt1;

                double sign = d21.cross(d31).z;

//                // If counter-clockwise:
//                if ( sign > 0 )
//                {

//                }

                k++;
            }
            associations.measurement.points[i];
            j++;
        }
        i++;
    }




}

void saveGraph(const Graph &graph, const std::string &filename)
{

}

}
