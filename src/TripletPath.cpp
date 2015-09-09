#include "triplet_graph/TripletPath.h"

namespace triplet_graph
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
            triplet_graph::Node* n_ptr = path.top();
            path.pop();
            str.append("\n"+n_ptr->id);
        }
    }

    return str;
}

}
