#include "triplet_graph/tools/rviz_tool.h"
#include "triplet_graph/Nodes.h"

#include <rviz/selection/selection_manager.h>
#include <rviz/display_context.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

namespace triplet_graph
{

// TODO: Rename because it is a more general tool for triplet graphs than just for setting edges to rigid
RigidEdgesTool::RigidEdgesTool():
    SelectionTool()
{
    shortcut_key_ = 'e';
    deleteClient_ = n_.serviceClient<triplet_graph::Nodes>("delete_nodes");
    mergeClient_ = n_.serviceClient<triplet_graph::Nodes>("merge_nodes");
    rigidEdgeClient_ = n_.serviceClient<triplet_graph::Nodes>("rigidify_edges");
}

int RigidEdgesTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
{
    std::vector<int> nodes;
    if ( event->type() == QKeyEvent::KeyPress)
    {
        // Obtain the selected nodes
        rviz::SelectionManager* sel_manager = context_->getSelectionManager();

        rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();

        int num_points = model->rowCount();

        triplet_graph::Nodes srv;

        for ( int i = 0; i < num_points; ++i )
        {
            QModelIndex child_index = model->index( i, 0 ); // get index of marker
            rviz::Property* child = model->getProp( child_index ); // get the marker
            std::string marker_id = child->getNameStd(); // get marker namespace/id
            int node_number = atoi(marker_id.substr( marker_id.find_last_of( "/" ) + 1 ).c_str()); // get marker id (node number)
            nodes.push_back(node_number);

            srv.request.nodes = nodes;
        }

        if ( nodes.size() )
        {
            // Node deletion
            if ( event->key() == Qt::Key_Delete )
            {
                std::cout << "Deleting node(s)" << std::endl;
                deleteClient_.call(srv);
            }

            // Node merger
            else if ( event->key() == Qt::Key_M )
            {
                std::cout << "Merging nodes" << std::endl;
                mergeClient_.call(srv);
            }

            // Setting edges to rigid
            else if ( event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return )
            {
                std::cout << "Setting all edges between selected nodes rigid" << std::endl;
                rigidEdgeClient_.call(srv);
            }
        }
    }
}

} // end namespace triplet_graph

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(triplet_graph::RigidEdgesTool,rviz::Tool )
// END_TUTORIAL
