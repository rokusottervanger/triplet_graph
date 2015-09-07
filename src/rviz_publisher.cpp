#include <ed_gui_server/EntityInfos.h>
#include <ed_gui_server/QueryMeshes.h>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <geolib/ros/msg_conversions.h>

#include <visualization_msgs/MarkerArray.h>

struct EntityViz
{
    EntityViz() : mesh_revision(0) {}

    unsigned int mesh_revision;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker text_marker;
    unsigned int num_id;
    unsigned int text_num_id;

};

std::map<std::string, EntityViz> entities;
ed_gui_server::QueryMeshes query_meshes_srv;

visualization_msgs::MarkerArray marker_msg;

double RATE = 10;

// ----------------------------------------------------------------------------------------------------

float COLORS[27][3] = { { 0.6, 0.6, 0.6},
                        { 0.6, 0.6, 0.4},
                        { 0.6, 0.6, 0.2},
                        { 0.6, 0.4, 0.6},
                        { 0.6, 0.4, 0.4},
                        { 0.6, 0.4, 0.2},
                        { 0.6, 0.2, 0.6},
                        { 0.6, 0.2, 0.4},
                        { 0.6, 0.2, 0.2},
                        { 0.4, 0.6, 0.6},
                        { 0.4, 0.6, 0.4},
                        { 0.4, 0.6, 0.2},
                        { 0.4, 0.4, 0.6},
                        { 0.4, 0.4, 0.4},
                        { 0.4, 0.4, 0.2},
                        { 0.4, 0.2, 0.6},
                        { 0.4, 0.2, 0.4},
                        { 0.4, 0.2, 0.2},
                        { 0.2, 0.6, 0.6},
                        { 0.2, 0.6, 0.4},
                        { 0.2, 0.6, 0.2},
                        { 0.2, 0.4, 0.6},
                        { 0.2, 0.4, 0.4},
                        { 0.2, 0.4, 0.2},
                        { 0.2, 0.2, 0.6},
                        { 0.2, 0.2, 0.4},
                        { 0.2, 0.2, 0.2}
                      };

// ----------------------------------------------------------------------------------------------------

unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for(unsigned int i = 0; i < str.size(); ++i)
        hash = ((hash << 5) + hash) + str[i]; /* hash * 33 + c */

    if (hash < 0)
        hash = -hash;

    return hash;
}

// ----------------------------------------------------------------------------------------------------

void initMarker(const std::string& id, visualization_msgs::Marker& m)
{
    m.color.a = 1;
    m.lifetime = ros::Duration(1.0 / RATE * 4);
    m.action = visualization_msgs::Marker::ADD;
    m.header.frame_id = "/map";

    int i_color = djb2(id) % 27;
    m.color.r = COLORS[i_color][0];
    m.color.g = COLORS[i_color][1];
    m.color.b = COLORS[i_color][2];
}

// ----------------------------------------------------------------------------------------------------

void meshToMarker(const ed_gui_server::Mesh& mesh, visualization_msgs::Marker& m)
{
    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    m.scale.x = m.scale.y = m.scale.z = 1.0;

    m.points.resize(mesh.vertices.size() / 3);
    for(unsigned int i = 0; i < m.points.size(); ++i )
    {
        unsigned int i3 = 3 * i;
        m.points[i].x = mesh.vertices[i3];
        m.points[i].y = mesh.vertices[i3 + 1];
        m.points[i].z = mesh.vertices[i3 + 2];
    }
}

// ----------------------------------------------------------------------------------------------------

void polygonToMarker(const ed_gui_server::EntityInfo& e, visualization_msgs::Marker& m)
{
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.scale.x = 0.01;

    for(unsigned int i = 0; i < e.polygon.xs.size(); ++i)
    {
        int j = (i + 1) % e.polygon.xs.size();

        float x1 = e.polygon.xs[i];
        float x2 = e.polygon.xs[j];

        float y1 = e.polygon.ys[i];
        float y2 = e.polygon.ys[j];


        geometry_msgs::Point p;

        // low line
        p.x = x1; p.y = y1; p.z = e.polygon.z_min;
        m.points.push_back(p);

        p.x = x2; p.y = y2; p.z = e.polygon.z_min;
        m.points.push_back(p);

        // high line
        p.x = x1; p.y = y1; p.z = e.polygon.z_max;
        m.points.push_back(p);

        p.x = x2; p.y = y2; p.z = e.polygon.z_max;
        m.points.push_back(p);

        // vertical line
        p.x = x1; p.y = y1; p.z = e.polygon.z_min;
        m.points.push_back(p);

        p.x = x1; p.y = y1; p.z = e.polygon.z_max;
        m.points.push_back(p);
    }
}

// ----------------------------------------------------------------------------------------------------

void entityCallback(const ed_gui_server::EntityInfos::ConstPtr& msg)
{
    for(unsigned int i = 0; i < msg->entities.size(); ++i)
    {
        const ed_gui_server::EntityInfo& info = msg->entities[i];

        if (info.id.size() >= 5 && info.id.substr(info.id.size() - 5) == "floor")
            continue; // Filter floor

        if (!info.has_pose)
            continue;

        EntityViz* entity_viz;

        std::map<std::string, EntityViz>::iterator it = entities.find(info.id);
        if (it == entities.end())
        {
            entity_viz = &entities[info.id];
            entity_viz->num_id = (entities.size() - 1) * 2;
            entity_viz->text_num_id = entity_viz->num_id + 1;

            initMarker(info.id, entity_viz->marker);
            entity_viz->marker.id = entity_viz->num_id;

            initMarker(info.id, entity_viz->text_marker);
            entity_viz->text_marker.id = entity_viz->text_num_id * 1000;
        }
        else
            entity_viz = &it->second;

        // HACK!: Check if this is a human
        if (info.color.a == 1 && info.color.r == 2 && info.color.g == 3 && (info.color.b == 4 || info.color.b == 5))
        {
            // This is a human
            marker_msg.markers.push_back(visualization_msgs::Marker());
            visualization_msgs::Marker& m = marker_msg.markers.back();

            initMarker(info.id, m);
            m.id = entity_viz->num_id;

            m.header.stamp = ros::Time::now();
            m.type = visualization_msgs::Marker::CYLINDER;

            m.color.a = 1.0;

            if (info.color.b == 4)
            {
                m.color.r = 0.2; m.color.g = 0.7; m.color.b = 1.0;  // human
            }
            else
            {
                m.color.r = 1.0; m.color.g = 0.7; m.color.b = 0.2;  // possible human (based on laser)
            }

            m.pose = info.pose;
            m.pose.position.z = .5;

            m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 1.0;
            m.ns = "entities";

            continue;
        }

        if (info.mesh_revision > entity_viz->mesh_revision)
        {
            query_meshes_srv.request.entity_ids.push_back(info.id);
            continue;
        }

        marker_msg.markers.push_back(entity_viz->marker);
        visualization_msgs::Marker& m = marker_msg.markers.back();

        // Set the pose and timestamp
        m.pose = info.pose;
        m.header.stamp = ros::Time::now();
        m.ns = "entities";

        // Set the color
        if (info.color.a != 0)
        {
            m.color.r = (float)info.color.r / 255;
            m.color.g = (float)info.color.g / 255;
            m.color.b = (float)info.color.b / 255;
        }

        m.color.a = info.existence_probability;

        if (info.mesh_revision == 0)
        {
            // Update polygon
            polygonToMarker(info, m);
        }

        // Add text
        marker_msg.markers.push_back(entity_viz->text_marker);
        visualization_msgs::Marker& m_text = marker_msg.markers.back();

        m_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        m_text.scale.x = m_text.scale.y = m_text.scale.z = 0.1;

        m_text.color.r = 0.9;
        m_text.color.g = 0.9;
        m_text.color.b = 0.9;

        m_text.pose = m.pose;
        m_text.pose.position.z += 0.1;
        m_text.header = m.header;
        m_text.ns = "entities";

//        if (type == "")
//            m.text = type + "(" + name.str().substr(0,4) +  ")";
//        else
//            m.text = name.str() + "(" + type.substr(0,4) +  ")";

//        std::stringstream ss_text;
//        ss_text << info.id.substr(0, 4);
//        if (!info.type.empty())
//            ss_text << " (" << info.type.substr(0, 4) << ")";

//        ss_text.precision(2);
//        ss_text << std::fixed << " (" << info.existence_probability << ")";

        if (info.type != "" && (!info.polygon.xs.empty() || info.mesh_revision > 0))
        {
            std::stringstream ss_text;
            ss_text << info.type;
            m_text.text = ss_text.str();
        }
        else
        {
            m_text.text = "";
        }
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_rviz_publisher");

    std::string pub_topic = "ed/rviz";
    if (argc > 1)
        pub_topic = argv[1];

    if (argc > 2)
        RATE = atof(argv[2]);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("ed/gui/entities", 1, entityCallback);

    ros::ServiceClient client = nh.serviceClient<ed_gui_server::QueryMeshes>("ed/gui/query_meshes");

    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(pub_topic, 1);

    ros::Rate r(RATE);
    while(ros::ok())
    {
        query_meshes_srv.request.entity_ids.clear();
        marker_msg.markers.clear();

        ros::spinOnce();

        if (!marker_msg.markers.empty())
            pub.publish(marker_msg);

        // Query missing meshes (if needed)
        if (!query_meshes_srv.request.entity_ids.empty())
        {
            if (client.call(query_meshes_srv))
            {

                for(unsigned int i = 0; i < query_meshes_srv.response.meshes.size(); ++i)
                {
                    const std::string& id = query_meshes_srv.response.entity_ids[i];
                    const ed_gui_server::Mesh& mesh = query_meshes_srv.response.meshes[i];

                    std::map<std::string, EntityViz>::iterator it = entities.find(id);
                    if (it == entities.end())
                        continue;

                    meshToMarker(mesh, it->second.marker);

                    it->second.mesh_revision = mesh.revision;
                }
            }
            else
            {
                ROS_ERROR("[ED RVIZ PUBLISHER] Could not query for meshes.");
            }
        }

        r.sleep();
    }

    return 0;
}
