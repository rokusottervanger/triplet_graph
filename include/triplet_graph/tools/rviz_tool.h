#ifndef TRIPLET_GRAPH_RVIZ_TOOL_H
#define TRIPLET_GRAPH_RVIZ_TOOL_H

#include <rviz/tool.h>
#include <rviz/default_plugin/tools/selection_tool.h>

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
    ~RigidEdgesTool();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
    virtual int processKeyEvent( QKeyEvent *event, rviz::RenderPanel *panel );

//    virtual void load( const rviz::Config& config );
//    virtual void save( rviz::Config config ) const;

private:
    bool selecting_;

};

} // end namespace triplet_graph

#endif // TRIPLET_GRAPH_RVIZ_TOOL_H
