#ifndef GRAPH_MAP_PATH_H_
#define GRAPH_MAP_PATH_H_

#include <stack>
#include <string>

#include "triplet_graph/Node.h"

namespace triplet_graph
{

class Path: public std::stack<int>
{
public:
    std::string toString();

    friend ostream& operator<<(ostream& os, const Date& dt);
};

ostream& operator<<(ostream& os, const Path& path)
{
    Path path_copy = path;
    os << "[ ";
    while ( !path_copy.empty() )
    {
        int top = path_copy.top();
        os << top << ' ';
        path_copy.pop();
    }
    os << "]";
    return os;
}

}

#endif
