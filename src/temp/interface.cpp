
// Basic ROS Include
#include <ros/ros.h>

// Standard Library Headers
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <vector>
//#include <map>
#include <string>

// MOOS Includes
#include <MOOS/libMOOS/App/MOOSApp.h>
#include "../riptide_ros_interface/moos_node.h"

// BOOST Includes
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace soslab;

MOOSNode MOOSApp;
void MOOSInit(const char * sMissionFile){
     const char * sMOOSName = "MOOS_ROS_BRIDGE";

     MOOSApp.Run(sMOOSName, sMissionFile);
}


int main(int argc, char **argv)
{
     if(argc < 3){
          ROS_INFO("Invalid number of parameters\n\n");
          ROS_INFO("argc is %d, but it should be at least 3.\n", argc);
          ROS_INFO("Usage:\n\trosrun moosros Bridge <moosrosconfig.xml> <mission.moos>");
          return 0;
     }

     //Initialize ROS Communications
     ros::init(argc, argv, "MOOS_ROS_Bridge");
     ros::NodeHandle n;

     //Check for existence of config files
     struct stat stFileInfo;
     int intStat;
     intStat = stat(argv[1],&stFileInfo);
     if(intStat != 0) {
          ROS_INFO("\n******\nCONFIG FILE MISSING\n%s does not exist!\n******\n",argv[1]);
          return 0;
     }

     ros::Rate loop_rate(10);

     //Kick off the MOOS Loop in a separate thread
     //before entering the ROS Loop
     boost::thread MOOSThread(MOOSInit, argv[2]);

     int count = 0;
     while (ros::ok())
     {
          ros::spinOnce();
          loop_rate.sleep();
          ++count;
     }

     return 0;
}
