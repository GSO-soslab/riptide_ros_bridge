
#include "ros_node.h"
#include "moos_node.h"

namespace soslab {

    void ROSNode::Initialize()
    {
        if(not m_moosNode)
        {
            throw std::runtime_error("MOOS Node should be set up before running");
        }


    }

    void ROSNode::wayPointCallback(const geometry_msgs::Point &msg)
    {
        const std::lock_guard<std::mutex> lock(m_pool->lock);


    }
}