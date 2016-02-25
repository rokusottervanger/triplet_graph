#include "triplet_graph/tools/rviz_tool.h"

#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/forwards.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

namespace triplet_graph
{

RigidEdgesTool::RigidEdgesTool()
{
    shortcut_key_ = 'e';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
RigidEdgesTool::~RigidEdgesTool()
{

}

// onInitialize() is called after construction (adding the tool to the
// toolbar)
void RigidEdgesTool::onInitialize()
{
//    SelectionTool::onInitialize();
    // Initialize service client?
}

// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void RigidEdgesTool::activate()
{
//    SelectionTool::activate();
}

// deactivate() is called when the tool is turned off because another
// tool has been chosen.
void RigidEdgesTool::deactivate()
{
//    SelectionTool::deactivate();
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
    if(event->type() == QKeyEvent::KeyPress)
    {
        if ( event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return )
        {
            rviz::SelectionManager* sel_manager = context_->getSelectionManager();

            rviz::M_Picked selection = sel_manager->getSelection();

            rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();

            int num_points = model->rowCount();

            for ( int i = 0; i < num_points; ++i )
            {
                QModelIndex child_index = model->index( i, 0 );
                rviz::Property* child = model->getProp( child_index );
                rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
                Ogre::Vector3 vec = subchild->getVector();
                std::cout << vec << std::endl;
            }

            // Selected nodes numbers are sent to the graph in a service call to fix all internal edges.
            std::cout << "Number of selected points: " << num_points << std::endl;
        }
    }
}

} // end namespace triplet_graph

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(triplet_graph::RigidEdgesTool,rviz::Tool )
// END_TUTORIAL


































