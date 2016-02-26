#include "triplet_graph/tools/rviz_tool.h"
#include "triplet_graph/Nodes.h"

#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/forwards.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

#include <std_srvs/Trigger.h>

#include <QDebug>

namespace triplet_graph
{

// TODO: Rename because it is a more general tool for triplet graphs than just for setting edges to rigid
RigidEdgesTool::RigidEdgesTool():
    SelectionTool()
{
    shortcut_key_ = 'e';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
RigidEdgesTool::~RigidEdgesTool(){}

// onInitialize() is called after construction (adding the tool to the
// toolbar)
void RigidEdgesTool::onInitialize()
{
    SelectionTool::onInitialize();
    // Initialize service client?
}

// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void RigidEdgesTool::activate()
{
    SelectionTool::activate();
}

// deactivate() is called when the tool is turned off because another
// tool has been chosen.
void RigidEdgesTool::deactivate()
{
    SelectionTool::deactivate();
    // Clear selection
}

// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
int RigidEdgesTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    return SelectionTool::processMouseEvent( event );
}

int RigidEdgesTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
{
    std::vector<int> nodes;
    if ( event->type() == QKeyEvent::KeyPress )
    {
        // Obtain the selected nodes
        rviz::SelectionManager* sel_manager = context_->getSelectionManager();

        rviz::M_Picked selection = sel_manager->getSelection();

        rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();

        int num_points = model->rowCount();

        for ( int i = 0; i < num_points; ++i )
        {
            QModelIndex child_index = model->index( i, 0 ); // get index of marker
            rviz::Property* child = model->getProp( child_index ); // get the marker
            std::string marker_id = child->getNameStd(); // get marker namespace/id
            int node_number = atoi(marker_id.substr( marker_id.find_last_of( "/" ) + 1 ).c_str()); // get marker id (node number)
            nodes.push_back(node_number);
        }

        // Node deletion
        if ( event->key() == Qt::Key_Delete )
        {
            std::cout << "Deleting node(s)" << std::endl;
        }

        // Node merger
        else if ( event->key() == Qt::Key_M )
        {
            std::cout << "Merging nodes" << std::endl;
        }

        // Setting edges to rigid
        else if ( event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return )
        {
            std::cout << "Setting all edges between selected nodes rigid" << std::endl;

            // Selected nodes numbers are sent to the graph in a service call to fix all internal edges.

        }
    }
}

} // end namespace triplet_graph

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(triplet_graph::RigidEdgesTool,rviz::Tool )
// END_TUTORIAL


































