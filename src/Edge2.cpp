#include "triplet_graph/Edge2.h"
#include <iostream>

namespace triplet_graph
{

int Edge2::tripletByNode(int n)
{
    std::map<int,int>::const_iterator it = triplet_by_node_.find(n);
    if ( it != triplet_by_node_.end() )
    {
        return it->second;
    }
    std::cout << "[EDGE2] tripletByNode: node not found, returning -1" << std::endl;
    return -1;
}

}
