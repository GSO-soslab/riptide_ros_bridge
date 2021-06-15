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
        /*! @brief: moos "NAV_ROLL" */
        double roll;
        /*! @brief: moos "NAV_PITCH" */
        double pitch;
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
        struct fill_c {
            unsigned x : 1;
            unsigned y : 1;
            unsigned z : 1;
            unsigned roll : 1;
            unsigned pitch : 1;
            unsigned yaw : 1;
            unsigned depth : 1;
            unsigned longitude : 1;
            unsigned latitude : 1;
            unsigned heading : 1;
            unsigned speed : 1;
        };
        fill_c _fill;
    } nav_t;

#define TEST_NAV_FILL(o) \
    o._fill.roll &         \
    o._fill.pitch &         \
    o._fill.yaw &         \
    o._fill.depth &       \
    o._fill.heading

    typedef struct waypoint_t {
        /*! @brief: moos "WPT_STAT" */
        std::string stat;
        /*! @brief: moos "WPT_ODO" */
        double odo;
        /*! @brief: moos "WPT_INDEX" */
        int index;
        /*! @brief: moos "WPT_UPDATE" */
        std::string update;

        struct fill_c {
            unsigned stat : 1;
            unsigned odo : 1;
            unsigned index : 1;
            unsigned update : 1;
        };
        fill_c _fill;
    } waypoint_t;


#define TEST_WAYPOINT_FILL(w) \
    w._fill.stat &             \
    w._fill.odo &              \
    w._fill.index &            \
    w._fill.update

    typedef struct imu_t {
        // 50hz
        float x_accel;
        float y_accel;
        float z_accel;
        float x_gyro;
        float y_gyro;
        float z_gyro;
        float x_vel;
        float y_vel;
        float z_vel;
        // 10hz
        float roll;
        float pitch;
        float yaw;
        float heading;
        // 1hz
        int status;
        int hw_status;
        int error;
        int self_test;
        int mode;
        int calib_sys;
        int calib_gyro;
        int calib_accel;
        int calib_mag;

        float w_quat;
        float x_quat;
        float y_quat;
        float z_quat;

        struct fill_c {
            unsigned roll : 1;
            unsigned pitch : 1;
            unsigned yaw : 1;
            unsigned heading : 1;
        };
        struct fill_extended_c {
            unsigned x_accel : 1;
            unsigned y_accel : 1;
            unsigned z_accel : 1;
            unsigned x_gyro : 1;
            unsigned y_gyro : 1;
            unsigned z_gyro : 1;
            unsigned w_quat : 1;
            unsigned x_quat : 1;
            unsigned y_quat : 1;
            unsigned z_quat : 1;
        };
        fill_c _fill;
        fill_extended_c _fill_extended;
    } imu_t;


#define TEST_IMU_FILL(w) \
    w._fill.roll & \
    w._fill.pitch & \
    w._fill.yaw & \
    w._fill.heading

#define TEST_IMU_FILL_EXTENDED(w) \
    w._fill_extended.x_accel &    \
    w._fill_extended.y_accel &    \
    w._fill_extended.z_accel &    \
    w._fill_extended.x_gyro &     \
    w._fill_extended.y_gyro &     \
    w._fill_extended.z_gyro &     \
    w._fill_extended.w_quat &     \
    w._fill_extended.x_quat &     \
    w._fill_extended.y_quat &     \
    w._fill_extended.z_quat

    typedef struct mag_t {
        float x;
        float y;
        float z;

        struct fill_c {
            unsigned x : 1;
            unsigned y : 1;
            unsigned z : 1;
        };
        fill_c _fill;
    } mag_t;

#define TEST_MAG_FILL(w) \
    w._fill.x &          \
    w._fill.y &          \
    w._fill.z

    typedef struct gps_t {
        // 1hz
        float latitude;
        float longitude;
        int quality;
        int sat;
        float hdop;
        int fix;
        float speed;
        float heading;
        float origin_latitude;
        float origin_longitude;
        float x;
        float y;
        int status;
        bool antenna_okay;
        int last_comms;
        int parse_errors;
        struct fill_c {
            unsigned fix : 1;
            unsigned sat : 1;
            unsigned parse_errors : 1;
            unsigned antenna_okay : 1;
            unsigned quality : 1 ;
        };
        fill_c _fill;
    } gps_t;

#define TEST_GPS_FILL(w) \
    w._fill.fix &        \
    w._fill.sat &        \
    w._fill.parse_errors & \
    w._fill.antenna_okay & \
    w._fill.quality

    //! @brief: Pressure message
    typedef struct ps_t {
        int status;
        int bat_temp;
        int bad_pressure;
        struct fill_c {
            unsigned status : 1;
            unsigned bad_temp : 1;
            unsigned bad_pressure : 1;
        };
        fill_c fill;
    } ps_t;

#define TEST_PS_FILL(w) \
    w._fill

    // todo: inspect for other variables related with depth
    typedef struct depth_t {
        float depth;
        std::string update;
        struct fill_c {
            unsigned depth : 1;
            unsigned update : 1;
        };
        fill_c fill;
    } depth_t;

#define TEST_DEPTH_FILL(w) \
    w._fill.depth & \
    w._fill.update


    typedef struct ivphelm_status_t {
        std::string state;
        std::map<std::string, bool> conditions;
        std::vector<std::string> condition_vars;
        std::map<std::string, std::string> update_vars;
        std::string allstop_msg;
        std::string update;
        bool manual_overide;
        struct fill_c {

        };
        fill_c fill;
    } ivphelm_status_t;

#define FLUSH_FILL(o) *((uint16_t*)&o._fill) = 0


    typedef struct pool_t {
        nav_t nav;

        waypoint_t waypoint;

        depth_t depth;

        gps_t gps;

        ps_t ps;

        imu_t imu;

        //!@note: microstrain imu
        imu_t ms_imu;

        ivphelm_status_t helm_status;

        std::mutex lock;
    } pool_t;
}

#endif
