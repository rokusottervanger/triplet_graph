#ifndef TRIPLET_GRAPH_RVIZ_TOOL_H
#define TRIPLET_GRAPH_RVIZ_TOOL_H

#include <rviz/tool.h>
#include <rviz/default_plugin/tools/selection_tool.h>

#include <ros/node_handle.h>

namespace triplet_graph
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class RigidEdgesTool: public rviz::SelectionTool
{
    Q_OBJECT
public:
    RigidEdgesTool();

    virtual int processKeyEvent( QKeyEvent *event, rviz::RenderPanel *panel );

private:
    ros::NodeHandle n_;
    ros::ServiceClient deleteClient_;
    ros::ServiceClient mergeClient_;
    ros::ServiceClient rigidEdgeClient_;

};

} // end namespace triplet_graph

#endif // TRIPLET_GRAPH_RVIZ_TOOL_H
