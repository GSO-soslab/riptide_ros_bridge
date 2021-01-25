#include "moos_node.h"
#include "pool.h"

#include <thread>
#include <functional>

#include "ros_node.h"
#include "utils.hpp"

using namespace soslab;

void MOOSNode::Init(const char *MissionFile)
{
    if(not m_rosNode)
    {
        throw std::runtime_error("ROS Node should be set up before running MOOS Node");
    }

    th = std::thread(
        [this, MissionFile] {
            Run("MOOS_ROS_BRIDGE", MissionFile);
        }
    );
}

bool MOOSNode::toMOOS(const std::string& moosName, double data)
{
    m_Comms.Notify(moosName, data, MOOSTime());
    return true;
}

bool MOOSNode::toMOOS(const std::string& moosName, std::string myString)
{
    m_Comms.Notify(moosName, myString, MOOSTime());
    return true;
}

//Support for binary-string from ROS to MOOS
bool MOOSNode::toMOOSBinaryString(const std::string& moosName, std::string myString)
{
    m_Comms.Notify(moosName, (void *)myString.c_str(), myString.size(), MOOSTime());
    return true;
}

bool MOOSNode::OnNewMail (MOOSMSG_LIST &NewMail)
{
    MOOSMSG_LIST::iterator p;
    for( p = NewMail.begin() ; p != NewMail.end() ; p++ )
    {

        // todo: parser for each sensor measurements
        // todo: parser for helm behaviours
        
        CMOOSMsg & rMsg = *p;

        Translate(rMsg);
        
        // SENSORY INFORMATION INPUT
        rMsg.GetTime();
        rMsg.GetType();
        rMsg.GetKey();
        rMsg.IsDouble();
        rMsg.IsString();

    }
    return true;
}

/*
   called by the base class when the application has made contact with
   the MOOSDB and a channel has been opened . Place code to specify what
   notifications you want to receive here .
*/
bool MOOSNode::OnConnectToServer()
{
    DoRegistrations();
    return true;
}

/*
  Called by the base class periodically. This is where you place code
  which does the work of the application
*/
bool MOOSNode::Iterate()
{
    return true;
}

/*
   called by the base class before the first :: Iterate is called . Place
   startup code here − especially code which reads configuration data from the
   mission file
*/
bool MOOSNode::OnStartUp()
{
    appTick = 1;
    commsTick = 1;

    if(!m_MissionReader.GetConfigurationParam("AppTick",appTick))
    {
        MOOSTrace("Warning, AppTick not set.\n");
    }

    if(!m_MissionReader.GetConfigurationParam("CommsTick",commsTick))
    {
        MOOSTrace("Warning, CommsTick not set.\n");
    }

    SetAppFreq(appTick);
    SetCommsFreq(commsTick);

    DoRegistrations();

    return true;
}

void MOOSNode::DoRegistrations()
{
    // todo: read them from config file
    auto list =
    {
    /*! @note: NAV  */
        "NAV_X",
        "NAV_Y",
        "NAV_Z",
        "NAV_YAW",
        "NAV_DEPTH",
        "NAV_HEADING",
        "NAV_LAT",
        "NAV_LONG",
        "NAV_SPEED",
    /*! @note: WAYPOINT  */
        "WPT_STAT",
        "WPT_ODO",
        "WPT_EFF_DIST_ALL",
        "WPT_EFF_DIST_LEG",
        "WPT_EFF_SUMM_ALL",
        "WPT_UPDATE",
    /*! @note: IVPHELM */
        "IVPHELM_STATEVARS",
        "IVPHELM_STATE",
        "IVPHELM_UPDATEVARS",
        "IVPHELM_ALLSTOP",
        "MOOS_MANUAL_OVERIDE"
    /*! @note: MOOS general */
    };

    for(const auto& i : list)
    {
        m_Comms.Register(i, 0);
        char buf[512];
        sprintf(buf, "Subscribing to %s\n", i);
        MOOSTrace(buf);
    }

}

void MOOSNode::Translate(CMOOSMsg &msg)
{
    const auto& key = msg.GetKey();
    const std::lock_guard<std::mutex> lock(m_pool->lock);
    // NAV
    if(        key == "NAV_X")
    {
        m_pool->nav.x = msg.GetDouble();
        m_pool->nav._fill.x |= 1;
    } else if (key == "NAV_Y")
    {
        m_pool->nav.y = msg.GetDouble();
        m_pool->nav._fill.y |= 1;
    } else if (key == "NAV_Z")
    {
        m_pool->nav.z = msg.GetDouble();
        m_pool->nav._fill.z |= 1;
    } else if (key == "NAV_YAW")
    {
        m_pool->nav.yaw = msg.GetDouble();
        m_pool->nav._fill.yaw |= 1;
    } else if (key == "NAV_DEPTH")
    {
        m_pool->nav.depth = msg.GetDouble();
        m_pool->nav._fill.depth |= 1;
    } else if (key == "NAV_HEADING")
    {
        m_pool->nav.heading = msg.GetDouble();
        m_pool->nav._fill.heading |= 1;
    } else if (key == "NAV_LAT")
    {
        m_pool->nav.latitude = msg.GetDouble();
        m_pool->nav._fill.latitude |= 1;
    } else if (key == "NAV_LONG")
    {
        m_pool->nav.longitude = msg.GetDouble();
        m_pool->nav._fill.longitude |= 1;
    } else if (key == "NAV_SPEED")
    {
        m_pool->nav.speed = msg.GetDouble();
        m_pool->nav._fill.speed |= 1;
    }
    
    // BHV WAYPOINT
    else if (        key == "WPT_STAT" )
    {
        m_pool->waypoint.stat = msg.GetAsString();
        m_pool->waypoint._fill.stat |= 1;
    } else if ( key == "WPT_ODO" )
    {
        m_pool->waypoint.odo = msg.GetDouble();
        m_pool->waypoint._fill.odo |= 1;
    } else if ( key == "WPT_UPDATE" )
    {
        m_pool->waypoint.update = msg.GetAsString();
        m_pool->waypoint._fill.update |= 1;
    } else if ( key == "WPT_INDEX" )
    {
        m_pool->waypoint.index = (int)msg.GetDouble();
        m_pool->waypoint._fill.index |= 1;
    }

    // IVPHELM
    else if ( key == "IVPHELM_STATE") {
        m_pool->helm_status.state = msg.GetAsString();
    } else if( key == "IVPHELM_STATEVARS") {
        m_pool->helm_status.condition_vars = parseCommaList(msg.GetAsString());
        for(const auto  i : m_pool->helm_status.condition_vars) {
            auto s = m_Comms.GetRegistered();
            auto r = s.find(i);
            if(r == s.end()) {
                m_Comms.Register(i, 0);
                char buf[512];
                sprintf(buf, "Subscribing to %s\n", i.c_str());
                MOOSTrace(buf);
            }
        }
    } else if ( key == "IVPHELM_UPDATEVARS") {
        m_pool->helm_status.update_vars = parseCommonMoosMsg(msg.GetAsString());
    } else if ( key == "IVPHELM_ALLSTOP" ) {
        m_pool->helm_status.allstop_msg = msg.GetAsString();
    }

    else if ( key == "MOOS_MANUAL_OVERIDE") {
        m_pool->helm_status.manual_overide = msg.GetAsString() == "true";
    }


    else {
        auto state_key_find = std::find(
                m_pool->helm_status.condition_vars.begin(),
                m_pool->helm_status.condition_vars.end(),
                key
                );
        if(state_key_find != m_pool->helm_status.condition_vars.end()) {
            m_pool->helm_status.conditions[key] = msg.GetAsString() == "true";
        }
    }

    if(TEST_NAV_FILL(m_pool->nav))
    {
        FLUSH_FILL(m_pool->nav);
        m_rosNode->PublishNav();
    }

    if(TEST_WAYPOINT_FILL(m_pool->waypoint))
    {
        FLUSH_FILL(m_pool->waypoint);

    }

}

bool MOOSNode::publishWayPointUpdate()
{
    return toMOOS("WPT_UPDATE", m_pool->waypoint.update);
}

bool MOOSNode::publishDepthUpdate()
{
    return toMOOS("DEP_UPDATE", m_pool->depth.update);
}

bool MOOSNode::publishIvpHelmUpdate(const std::string& name, bool state)
{
    return toMOOS(name, state ? "true" : "false");
}

bool MOOSNode::publishManualOveride(bool state) {
    return toMOOS("MOOS_MANUAL_OVERIDE", state ? "true" : "false");
}
