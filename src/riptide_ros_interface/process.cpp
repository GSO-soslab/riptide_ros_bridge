#include <ros/ros.h>
#include <ros/package.h>
#include "process.h"

namespace soslab {

    Process::Process() :
        m_rosNode(std::make_shared<ROSNode>()),
        m_moosNode(std::make_shared<MOOSNode>()),
        m_pool(std::make_shared<pool_t>())
    {
        m_moosNode->SetPool(m_pool);

        m_rosNode->SetPool(m_pool);
        m_rosNode->SetMoosNode(m_moosNode);
    }

    void Process::Run(int argc, char* argv[])
    {
        char *missionFile;
        for(int c = 0; c != -1 ; c = getopt(argc, argv, "m:") )
        {
            switch (c) {
                case 'm':
                    missionFile = optarg;
                    break;
                case '?':
                    if (optopt == 'm') {
                        fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                    }
                default:
                    break;
            }
        }

        // start moos
        m_moosNode->Init(missionFile);

        // start ros
        ros::init(argc, argv, "moos_ros_bridge");
        m_rosNode->SetPool(m_pool);
        m_rosNode->Initialize();

        ros::spin();

    }

}