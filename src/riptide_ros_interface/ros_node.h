#ifndef ROSNODE_H_
#define ROSNODE_H_

#include <utility>
#include <thread>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include "pool.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
// MSG
#include "riptide_ros_interface/Nav.h"
#include "riptide_ros_interface/IvpHelmState.h"

// SRV
#include "riptide_ros_interface/SetWayPoint.h"
#include "riptide_ros_interface/SetDepth.h"
#include "riptide_ros_interface/SetHelmCondition.h"


namespace soslab {


    class MOOSNode;

    class ROSNode {
    private:
        riptide_ros_interface::Nav m_nav_msg;
        sensor_msgs::Imu m_imu_msg;

        riptide_ros_interface::IvpHelmState m_ivp_helm_state_msg;

    protected:

        ros::NodeHandle m_nh;
        ros::NodeHandle m_pnh;

        ros::Publisher m_nav_publisher;

        ros::Publisher m_imu_publisher;

        ros::Publisher m_ivp_helm_state_publisher;

        /** @brief: Receive and transmit way point to moos waypoint behavior */
        ros::ServiceServer m_wpt_service;

        /** @brief: Receive and transmit depth to moos constant depth behavior */
        ros::ServiceServer m_dep_service;

        /** @brief: Set helm state */
        ros::ServiceServer m_helm_state_service;

        /** @brief: m_manual_overide_service */
        ros::ServiceServer m_manual_overide_service;

        std::shared_ptr<MOOSNode> m_moosNode;

        std::shared_ptr<pool_t> m_pool;

        bool wayPointService(riptide_ros_interface::SetWayPoint::Request& req,
                             riptide_ros_interface::SetWayPoint::Response& res);

        bool depthService(riptide_ros_interface::SetDepth::Request& req,
                          riptide_ros_interface::SetDepth::Response& res);

        bool ivpHelmConditionService(riptide_ros_interface::SetHelmCondition::Request& req,
                                     riptide_ros_interface::SetHelmCondition::Response& res);

        bool manualOverideService(std_srvs::SetBool::Request& req,
                                  std_srvs::SetBool::Response& res);

    public:

        ROSNode();

        void Initialize();

        void SetMoosNode(std::shared_ptr<MOOSNode> n) { m_moosNode = n; }

        void SetPool(std::shared_ptr<pool_t> p) { m_pool = p; }

        void PublishNav();

        void PublishIvpHelmState();

        void PublishImu();


    };
    
}

#endif //RIPTIDE_ROS_INTERFACE_ROSNODE_H
