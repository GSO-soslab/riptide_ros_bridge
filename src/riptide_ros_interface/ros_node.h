#ifndef ROSNODE_H_
#define ROSNODE_H_

#include <utility>

#include "ros/ros.h"

#include "pool.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
// MSG
#include "riptide_ros_interface/Nav.h"

// SRV
#include "riptide_ros_interface/WayPoint.h"
#include "riptide_ros_interface/Depth.h"

namespace soslab {


    class MOOSNode;

    class ROSNode {
    private:
        riptide_ros_interface::Nav m_nav_msg;
        sensor_msgs::Imu m_imu_msg;

    protected:

        ros::NodeHandle m_nh;
        ros::NodeHandle m_pnh;

        ros::Publisher m_nav_publisher;

        /** @brief: Receive and transmit way point to moos waypoint behavior */
        ros::ServiceServer m_wpt_service;

        /** @brief: Receive and transmit depth to moos constant depth behavior */
        ros::ServiceServer m_dep_service;

        bool wayPointService(riptide_ros_interface::WayPoint::Request& req,
                             riptide_ros_interface::WayPoint::Response& res);

        bool depthService(riptide_ros_interface::Depth::Request& req,
                          riptide_ros_interface::Depth::Response& res);

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
