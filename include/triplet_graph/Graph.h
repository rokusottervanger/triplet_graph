#ifndef TRIPLET_GRAPH_GRAPH_H_
#define TRIPLET_GRAPH_GRAPH_H_

#include <vector>
#include <algorithm>

#include "Node.h"
#include "Edge2.h"
#include "Edge3.h"
#include "Measurement.h"

namespace triplet_graph
{

class Graph
{
private:
    std::vector<Node> nodes_;
    std::vector<Edge2> edges_;
    std::vector<Edge3> triplets_;

    AssociatedMeasurement last_associations_;

    std::vector<int> deleted_nodes_;
    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;

public:
    Graph(){}

    ~Graph(){}

    int addNode(const std::string &id);

    int addEdge2(const int n1, const int n2, const double& length);

    int addEdge3(const int, const int, const int);

    void deleteNode(const int i);

    void deleteEdge2(const int i);

    void deleteEdge3(const int i);

    void setEdgeLength(const int i, const double l);

    void flipTriplet(const int i);

    void setAssociations(const AssociatedMeasurement& associations) { last_associations_ = associations; }

    AssociatedMeasurement getAssociations() const { return last_associations_; }

    // TODO: This still contains the deleted nodes, edges and triplets!
    std::vector<Node> getNodes() const {return nodes_;}
    std::vector<Edge2> getEdge2s() const {return edges_;}
    std::vector<Edge3> getEdge3s() const {return triplets_;}


    // TODO: implement iterators for edges and triplets
    class NodeIterator : public std::iterator<std::forward_iterator_tag, Node>
    {
        public:

            NodeIterator(const std::vector<Node>& v) : it_(v.begin()), it_end_(v.end())
            {
                // Skip possible deleted nodes at the beginning
                while(it_ != it_end_ && it_->id == "")
                    ++it_;
            }

            NodeIterator(const NodeIterator& it) : it_(it.it_) {}

            NodeIterator(const std::vector<Node>::const_iterator& it) : it_(it) {}

            NodeIterator& operator++()
            {
                // Increase iterator and skip possible zero-entities (deleted entities)
                do { ++it_; if (it_ == it_end_) break; } while ( it_->id == "");
                return *this;
            }

            NodeIterator operator++(int) { NodeIterator tmp(*this); operator++(); return tmp; }

            bool operator==(const NodeIterator& rhs) { return it_ == rhs.it_; }

            bool operator!=(const NodeIterator& rhs) { return it_ != rhs.it_; }

            const Node& operator*() { return *it_; }

            int operator-(const NodeIterator& rhs) { return it_ - rhs.it_; }

            NodeIterator operator+=(const int offset) {
                it_ += offset;
                return it_;
            }

            friend NodeIterator operator+(NodeIterator it, const int offset) { return it+=offset; }

            const Node* operator->() const { return &*it_; }

        private:

            std::vector<Node>::const_iterator it_;
            std::vector<Node>::const_iterator it_end_;
    };

    typedef NodeIterator const_iterator;

    inline const_iterator begin() const { return const_iterator(nodes_); }

    inline const_iterator end() const { return const_iterator(nodes_.end());}

    inline int size() const { return nodes_.size() - deleted_nodes_.size(); }

};

}

#endif
