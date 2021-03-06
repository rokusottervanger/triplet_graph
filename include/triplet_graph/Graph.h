#ifndef TRIPLET_GRAPH_GRAPH_H_
#define TRIPLET_GRAPH_GRAPH_H_

#include <vector>
#include <algorithm>

#include "Node.h"
#include "Edge2.h"
#include "Edge3.h"
#include "Measurement.h"
#include "Server.h"

namespace triplet_graph
{

class Graph
{
private:
    std::vector<Node> nodes_;
    std::vector<Edge2> edges_;
    std::vector<Edge3> triplets_;

    std::vector<int> deleted_nodes_;
    std::vector<int> deleted_edges_;
    std::vector<int> deleted_triplets_;

    Server guiServer_;

public:
    Graph(): guiServer_(this) {}

    ~Graph(){}

    int addNode(const std::string &id);

    int addEdge2(const int n1, const int n2, const double length, const double std_dev);

    int addEdge3(const int, const int, const int);

    void deleteNode(const int i);

    void deleteEdge2(const int i);

    void deleteEdge3(const int i);

    void setEdgeLength(const int i, const double l);

    void setEdgeRigid(const int i);

    void setEdgeRigid(const int n1, const int n2);

//    void mergeNodes(const int n1, const int n2);

    void flipTriplet(const int i);

    // -----------------------------------------------------------------------------------------------

    class NodeIterator : public std::iterator<std::forward_iterator_tag, Node>
    {
        public:

            NodeIterator(const std::vector<Node>& v) : it_(v.begin()), it_end_(v.end())
            {
                // Skip possible deleted nodes at the beginning
                while(it_ != it_end_ && it_->deleted)
                    ++it_;
            }

            NodeIterator(const NodeIterator& it) : it_(it.it_) {}

            NodeIterator(const std::vector<Node>::const_iterator& it) : it_(it) {}

            NodeIterator& operator++()
            {
                // Increase iterator and skip possible zero-entities (deleted entities)
                do { ++it_; if (it_ == it_end_) break; } while ( it_->deleted);
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

    // -----------------------------------------------------------------------------------------------

    class Edge2Iterator : public std::iterator<std::forward_iterator_tag, Edge2>
    {
        public:

            Edge2Iterator(const std::vector<Edge2>& v) : it_(v.begin()), it_end_(v.end())
            {
                // Skip possible deleted edges at the beginning
                while(it_ != it_end_ && it_->deleted)
                    ++it_;
            }

            Edge2Iterator(const Edge2Iterator& it) : it_(it.it_) {}

            Edge2Iterator(const std::vector<Edge2>::const_iterator& it) : it_(it) {}

            Edge2Iterator& operator++()
            {
                // Increase iterator and skip possible zero-entities (deleted entities)
                do { ++it_; if (it_ == it_end_) break; } while ( it_->deleted);
                return *this;
            }

            Edge2Iterator operator++(int) { Edge2Iterator tmp(*this); operator++(); return tmp; }

            bool operator==(const Edge2Iterator& rhs) { return it_ == rhs.it_; }

            bool operator!=(const Edge2Iterator& rhs) { return it_ != rhs.it_; }

            const Edge2& operator*() { return *it_; }

            int operator-(const Edge2Iterator& rhs) { return it_ - rhs.it_; }

            Edge2Iterator operator+=(const int offset) {
                it_ += offset;
                return it_;
            }

            friend Edge2Iterator operator+(Edge2Iterator it, const int offset) { return it+=offset; }

            const Edge2* operator->() const { return &*it_; }

        private:

            std::vector<Edge2>::const_iterator it_;
            std::vector<Edge2>::const_iterator it_end_;
    };

    typedef Edge2Iterator const_edge2_iterator;

    inline const_edge2_iterator beginEdges() const { return const_edge2_iterator(edges_); }

    inline const_edge2_iterator endEdges() const { return const_edge2_iterator(edges_.end());}

    inline int numEdges() const { return edges_.size() - deleted_edges_.size(); }

    // -----------------------------------------------------------------------------------------------

    class Edge3Iterator : public std::iterator<std::forward_iterator_tag, Edge3>
    {
        public:

            Edge3Iterator(const std::vector<Edge3>& v) : it_(v.begin()), it_end_(v.end())
            {
                // Skip possible deleted edges at the beginning
                while(it_ != it_end_ && it_->deleted)
                    ++it_;
            }

            Edge3Iterator(const Edge3Iterator& it) : it_(it.it_) {}

            Edge3Iterator(const std::vector<Edge3>::const_iterator& it) : it_(it) {}

            Edge3Iterator& operator++()
            {
                // Increase iterator and skip any deleted entities
                do { ++it_; if (it_ == it_end_) break; } while ( it_->deleted );
                return *this;
            }

            Edge3Iterator operator++(int) { Edge3Iterator tmp(*this); operator++(); return tmp; }

            bool operator==(const Edge3Iterator& rhs) { return it_ == rhs.it_; }

            bool operator!=(const Edge3Iterator& rhs) { return it_ != rhs.it_; }

            const Edge3& operator*() { return *it_; }

            int operator-(const Edge3Iterator& rhs) { return it_ - rhs.it_; }

            Edge3Iterator operator+=(const int offset) {
                it_ += offset;
                return it_;
            }

            friend Edge3Iterator operator+(Edge3Iterator it, const int offset) { return it+=offset; }

            const Edge3* operator->() const { return &*it_; }

        private:

            std::vector<Edge3>::const_iterator it_;
            std::vector<Edge3>::const_iterator it_end_;
    };

    typedef Edge3Iterator const_edge3_iterator;

    inline const_edge3_iterator beginTriplets() const { return const_edge3_iterator(triplets_); }

    inline const_edge3_iterator endTriplets() const { return const_edge3_iterator(triplets_.end());}

    inline int numTriplets() const { return triplets_.size() - deleted_triplets_.size(); }

};

}

#endif
