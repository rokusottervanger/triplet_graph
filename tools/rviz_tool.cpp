#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "triplet_graph/tools/rviz_tool.h"

namespace rviz_plugin_tutorials
{

RigidEdgesTool::RigidEdgesTool():
    moving_flag_node_( NULL ),
    current_flag_property_( NULL )
{
    shortcut_key_ = 'l';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
RigidEdgesTool::~RigidEdgesTool()
{
    for( unsigned i = 0; i < flag_nodes_.size(); i++ )
    {
        scene_manager_->destroySceneNode( flag_nodes_[ i ]);
    }
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void RigidEdgesTool::onInitialize()
{
    flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";

    if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
    {
        ROS_ERROR( "PlantFlagTool: failed to load model resource '%s'.", flag_resource_.c_str() );
        return;
    }

    moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    moving_flag_node_->attachObject( entity );
    moving_flag_node_->setVisible( false );
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void RigidEdgesTool::activate()
{
    if( moving_flag_node_ )
    {
        moving_flag_node_->setVisible( true );

        current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
        current_flag_property_->setReadOnly( true );
        getPropertyContainer()->addChild( current_flag_property_ );
    }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.
void RigidEdgesTool::deactivate()
{
    if( moving_flag_node_ )
    {
        moving_flag_node_->setVisible( false );
        delete current_flag_property_;
        current_flag_property_ = NULL;
    }
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int RigidEdgesTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    if( !moving_flag_node_ )
    {
        return Render;
    }
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                           ground_plane,
                                           event.x, event.y, intersection ))
    {
        moving_flag_node_->setVisible( true );
        moving_flag_node_->setPosition( intersection );
        current_flag_property_->setVector( intersection );

        if( event.leftDown() )
        {
            // Start drawing box and selecting nodes when inside box
            makeFlag( intersection );
            current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
            return Render | Finished;
        }
    }
    else
    {
        moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
    }
    return Render;
}

// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void RigidEdgesTool::makeFlag( const Ogre::Vector3& position )
{
    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );
    flag_nodes_.push_back( node );
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::RigidEdgesTool,rviz::Tool )
// END_TUTORIAL
