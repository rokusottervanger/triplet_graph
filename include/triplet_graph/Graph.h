#ifndef TRIPLET_GRAPH_GRAPH_H_
#define TRIPLET_GRAPH_GRAPH_H_

#include <list>
#include <geolib/datatypes.h>
#include <tue/config/configuration.h>
#include <triplet_graph/Measurement.h>

#include "Edge2.h"
#include "Edge3.h"
#include "Node.h"

namespace triplet_graph
{

class Graph
{
public:
    Graph(){}

    ~Graph(){}

    int addNode(const std::string &id);

    int addEdge2(const int&, const int&, double &);

    int addEdge3(const int&, const int&, const int&);

    void deleteNode(const int& i);

    // TODO: methods to remove edges

//    bool configure(tue::Configuration &config);

    void update(const Measurements& measurements);

    std::vector<Node> getNodes() const {return nodes_;}
    std::vector<Edge2> getEdge2s() const {return edges_;}
    std::vector<Edge3> getEdge3s() const {return triplets_;}

    class NodeIterator : public std::iterator<std::forward_iterator_tag, Node>
    {
        public:

            NodeIterator(const std::vector<Node>& v) : it_(v.begin()), it_end_(v.end())
            {
                // Skip possible deleted nodes at the beginning
                while(it_ != it_end_ && it_->id != "")
                    ++it_;
            }

            NodeIterator(const NodeIterator& it) : it_(it.it_) {}

            NodeIterator(const std::vector<Node>::const_iterator& it) : it_(it) {}

            NodeIterator& operator++()
            {
                // Increase iterator and skip possible zero-entities (deleted entities)
                do { ++it_; if (it_ == it_end_) break; } while ( it_->id != "");
                return *this;
            }

            NodeIterator operator++(int) { NodeIterator tmp(*this); operator++(); return tmp; }

            bool operator==(const NodeIterator& rhs) { return it_ == rhs.it_; }

            bool operator!=(const NodeIterator& rhs) { return it_ != rhs.it_; }

            const Node& operator*() { return *it_; }

        private:

            std::vector<Node>::const_iterator it_;
            std::vector<Node>::const_iterator it_end_;
    };


private:
    std::vector<Node> nodes_;
    std::vector<Edge2> edges_;
    std::vector<Edge3> triplets_;

    std::vector<int> deleted_nodes_;
    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;
};

}

#endif
