#include "triplet_graph/Path.h"

namespace triplet_graph
{

std::ostream& operator<<(std::ostream& os, const Path& path)
{
    os << "[ ";
    for ( Path::const_iterator it = path.begin(); it != path.end(); it++ )
        os << *it << ' ';
    os << "]";
    return os;
}

}
