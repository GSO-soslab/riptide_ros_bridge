#if !defined(POOL_H_)
#define POOL_H_

#include <string>
#include <mutex>

namespace soslab {
    typedef struct nav_t {
        /*! @brief: moos "NAV_X" */
        double x;
        /*! @brief: moos "NAV_Y" */
        double y;
        /*! @brief: moos "NAV_Z" */
        double z;
        /*! @brief: moos "NAV_YAW" */
        double yaw;
        /*! @brief: moos "NAV_LONG" */
        double depth;
        /*! @brief: moos "NAV_LONG" */
        double longitude;
        /*! @brief: moos "NAV_LAT" */
        double latitude;
        /*! @brief: moos "NAV_HEADING" */
        double heading;
        /*! @brief: moos "NAV_SPEED" */
        double speed;
    } nav_t;

    typedef struct nav_fill_t {
        unsigned x : 1;
        unsigned y : 1;
        unsigned z : 1;
        unsigned yaw : 1;
        unsigned depth : 1;
        unsigned longitude : 1;
        unsigned latitude : 1;
        unsigned heading : 1;
        unsigned speed : 1;
    } nav_fill_t;

#define TEST_NAV_FILL(o) \
    o.x &           \
    o.y &           \
    o.z &           \
    o.yaw &         \
    o.depth &       \
    o.longitude &   \
    o.latitude &    \
    o.heading &     \
    o.speed

#define FLUSH_FILL(o) *((uint16_t*)&o) = 0

    typedef struct waypoint_t {
        /*! @brief: moos "WPT_STAT" */
        std::string stat;
        /*! @brief: moos "WPT_ODO" */
        double odo;
        /*! @brief: moos "WPT_INDEX" */
        int index;
        /*! @brief: moos "WPT_UPDATE" */
        std::string update;
        
    } waypoint_t;

    typedef struct waypoint_fill_t {
        unsigned stat : 1;
        unsigned odo : 1;
        unsigned index : 1;
        unsigned update : 1;
    } waypoint_fill_t;

#define TEST_WAYPOINT_FILL(w) \
    w.stat &             \
    w.odo &              \
    w.index &            \
    w.update

    typedef struct pool_t {
        nav_t nav;
        nav_fill_t nav_fill;

        waypoint_t waypoint;
        waypoint_fill_t  waypoint_fill;

        std::mutex lock;
    } pool_t;
}

#endif
