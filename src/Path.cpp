#include "triplet_graph/Path.h"

namespace triplet_graph
{

std::ostream& operator<<(std::ostream& os, const Path& path)
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
