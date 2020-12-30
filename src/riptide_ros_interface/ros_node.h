#ifndef ROSNODE_H_
#define ROSNODE_H_

#include <utility>

#include "ros/ros.h"

#include "pool.h"
#include "geometry_msgs/Point.h"
#include "riptide_ros_interface/Nav.h"

namespace soslab {


    class MOOSNode;

    class ROSNode {
    private:
        riptide_ros_interface::Nav m_nav_msg;

    protected:

        ros::NodeHandle m_nh;
        ros::NodeHandle m_pnh;

        ros::Publisher m_nav_publisher;

        void wayPointCallback(const geometry_msgs::Point& msg);
        std::shared_ptr<MOOSNode> m_moosNode;
        std::shared_ptr<pool_t> m_pool;
    public:

        ROSNode();

        void Initialize();

        void SetMoosNode(std::shared_ptr<MOOSNode> n) { m_moosNode = n; }

        void SetPool(std::shared_ptr<pool_t> p) { m_pool = p; }

        void PublishNav();
    };
    
}

#endif //RIPTIDE_ROS_INTERFACE_ROSNODE_H
