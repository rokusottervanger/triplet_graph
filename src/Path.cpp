#include "triplet_graph/Path.h"

namespace triplet_graph
{

std::ostream& operator<<(std::ostream& os, const Path& path)
{
    os << "[ ";
    int i = 0;
    for ( Path::const_iterator it = path.begin(); it != path.end(); it++ )
    {
        os << *it << " (" << path.parent_tree[i].first << "," << path.parent_tree[i].second << ") (" << path.costs[i] << "); ";
        ++i;
    }

    os << "]";
    return os;
}

}
