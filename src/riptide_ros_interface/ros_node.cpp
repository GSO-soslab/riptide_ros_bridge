
#include "ros_node.h"
#include "moos_node.h"

namespace soslab {

    ROSNode::ROSNode() :
        m_nh(),
        m_pnh("~")
    {
        m_nav_msg.header.seq = 0;

        m_nav_publisher = m_pnh.advertise<riptide_ros_interface::Nav>("navigation", 1000);

        m_wpt_service = m_pnh.advertiseService("send_waypoint", &ROSNode::wayPointService, this);

        m_dep_service = m_pnh.advertiseService("send_depth", &ROSNode::depthService, this);


    }

    void ROSNode::Initialize()
    {
        if(not m_moosNode)
        {
            throw std::runtime_error("MOOS Node should be set up before running ROS Node");
        }
    }

    bool ROSNode::wayPointService(riptide_ros_interface::WayPointRequest& req,
                                  riptide_ros_interface::WayPointResponse& res) {
        const std::lock_guard<std::mutex> lock(m_pool->lock);

        if(req.list.empty()) {
            return false;
        } else {
            m_pool->waypoint.update.clear();
            m_pool->waypoint.update += "points = pts={";
            for(const auto& point : req.list) {
                m_pool->waypoint.update += std::to_string(point.x) + "," + std::to_string(point.y) + ":";
            }
            m_pool->waypoint.update += "}";
        }
        return m_moosNode->publishWayPointUpdate();
    }

    bool ROSNode::depthService(riptide_ros_interface::Depth::Request &req,
                               riptide_ros_interface::Depth::Response &res) {
        const std::lock_guard<std::mutex> lock(m_pool->lock);

        m_pool->depth.update = "depth = " + std::to_string(req.depth);

        return m_moosNode->publishDepthUpdate();
    }

    void ROSNode::PublishNav() {

        m_nav_msg.header.seq += 1;
        m_nav_msg.header.stamp = ros::Time::now();
        m_nav_msg.x = m_pool->nav.x;
        m_nav_msg.y = m_pool->nav.y;
        m_nav_msg.z = m_pool->nav.z;
        m_nav_msg.depth = m_pool->nav.depth;
        m_nav_msg.longitude = m_pool->nav.longitude;
        m_nav_msg.latitude = m_pool->nav.latitude;
        m_nav_msg.heading = m_pool->nav.heading;
        m_nav_msg.speed = m_pool->nav.speed;

        m_nav_publisher.publish(m_nav_msg);
    }
}