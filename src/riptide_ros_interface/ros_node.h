#ifndef ROSNODE_H_
#define ROSNODE_H_

#include <utility>

#include "pool.h"
#include "geometry_msgs/Point.h"


namespace soslab {


    class MOOSNode;

    class ROSNode {
    private:
        void wayPointCallback(const geometry_msgs::Point& msg);
        std::shared_ptr<MOOSNode> m_moosNode;
        std::shared_ptr<pool_t> m_pool;
    public:

        ROSNode() = default;

        void Initialize();

        void SetMoosNode(std::shared_ptr<MOOSNode> n) { m_moosNode = n; }

        void SetPool(std::shared_ptr<pool_t> p) { m_pool = p; }
    };
    
}

#endif //RIPTIDE_ROS_INTERFACE_ROSNODE_H
