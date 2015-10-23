#include "triplet_graph/Path.h"

namespace triplet_graph
{

//std::string Path::toString()
//{
//    std::string str("");
//    Path path = *this;

//    if (this->size() == 0)
//        str.append("Path does not contain any nodes");
//    else
//    {
//        str.append("Path contains these nodes:");

//        while (!path.empty())
//        {
//            triplet_graph::Node* n_ptr = path.top();
//            path.pop();
//            str.append("\n"+n_ptr->id);
//        }
//    }

//    return str;
//}

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
