#include "moos_node.h"
#include "pool.h"

#include <thread>
#include <functional>

using namespace soslab;

void MOOSNode::Init(const char *MissionFile)
{
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
    std::vector<std::string> list ={
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
        "WPA_UPDATE",
    };

    for(const auto& i : list)
    {
        m_Comms.Register(i, 0);
        char buf[512];
        sprintf(buf, "Subscribing to %s\n", i.c_str());
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
        m_pool->nav_fill.x |= 1;
    } else if (key == "NAV_Y")
    {
        m_pool->nav.y = msg.GetDouble();
        m_pool->nav_fill.y |= 1;
    } else if (key == "NAV_Z")
    {
        m_pool->nav.z = msg.GetDouble();
        m_pool->nav_fill.z |= 1;
    } else if (key == "NAV_YAW")
    {
        m_pool->nav.yaw = msg.GetDouble();
        m_pool->nav_fill.yaw |= 1;
    } else if (key == "NAV_DEPTH")
    {
        m_pool->nav.depth = msg.GetDouble();
        m_pool->nav_fill.depth |= 1;
    } else if (key == "NAV_HEADING")
    {
        m_pool->nav.heading = msg.GetDouble();
        m_pool->nav_fill.heading |= 1;
    } else if (key == "NAV_LAT")
    {
        m_pool->nav.latitude = msg.GetDouble();
        m_pool->nav_fill.latitude |= 1;
    } else if (key == "NAV_LONG")
    {
        m_pool->nav.longitude = msg.GetDouble();
        m_pool->nav_fill.longitude |= 1;
    } else if (key == "NAV_SPEED")
    {
        m_pool->nav.speed = msg.GetDouble();
        m_pool->nav_fill.speed |= 1;
    }
    
    // BHV WAYPOINT
    if (        key == "WPT_STAT" )
    {
        m_pool->waypoint.stat = msg.GetAsString();
        m_pool->waypoint_fill.stat |= 1;
    } else if ( key == "WPT_ODO" )
    {
        m_pool->waypoint.odo = msg.GetDouble();
        m_pool->waypoint_fill.odo |= 1;
    } else if ( key == "WPT_UPDATE" )
    {
        m_pool->waypoint.update = msg.GetAsString();
        m_pool->waypoint_fill.update |= 1;
    } else if ( key == "WPT_INDEX" )
    {
        m_pool->waypoint.index = (int)msg.GetDouble();
        m_pool->waypoint_fill.index |= 1;
    }
    
    // BHV GO TO DEPTH


    if(TEST_NAV_FILL(m_pool->nav_fill))
    {

    }

    if(TEST_WAYPOINT_FILL(m_pool->waypoint_fill))
    {

    }
}

bool MOOSNode::publishWayPoint(waypoint_t w)
{

    return toMOOS("WPT_UPDATE", w.update);

}
