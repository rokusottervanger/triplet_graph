#include "graph_map/Path.h"

namespace graph_map
{

std::string Path::toString()
{
    std::string str("");
    Path path = *this;

    if (this->size() == 0)
        str.append("Path does not contain any nodes");
    else
    {
        str.append("Path contains these nodes:");

        while (!path.empty())
        {
            graph_map::Node* n_ptr = path.top();
            path.pop();
            str.append("\n"+n_ptr->id);
        }
    }

    return str;
}

}
