#include "triplet_graph/Path.h"

namespace triplet_graph
{

std::ostream& operator<<(std::ostream& os, Path path)
{
    os << "[ ";
    while ( !path.empty() )
    {
        int top = path.top();
        os << top << ' ';
        path.pop();
    }
    os << "]";
    return os;
}

}
