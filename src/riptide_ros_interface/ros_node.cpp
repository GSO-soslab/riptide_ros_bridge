
#include "ros_node.h"
#include "moos_node.h"
#include "auv_msgs/NavigationStatus.h"
#include "riptide_ros_interface/Nav.h"

namespace soslab {

    ROSNode::ROSNode() :
        m_nh(),
        m_pnh("~")
    {
        m_nav_msg.header.seq = 0;

        m_nav_publisher = m_pnh.advertise<riptide_ros_interface::Nav>("navigation", 1000);
    }

    void ROSNode::Initialize()
    {
        if(not m_moosNode)
        {
            throw std::runtime_error("MOOS Node should be set up before running ROS Node");
        }
    }

    void ROSNode::wayPointCallback(const geometry_msgs::Point &msg)
    {
        const std::lock_guard<std::mutex> lock(m_pool->lock);


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