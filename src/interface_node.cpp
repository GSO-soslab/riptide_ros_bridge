#include "riptide_ros_interface/ros_node.h"
#include "riptide_ros_interface/process.h"


int main(int argc, char* argv[]) {

    soslab::Process p;
    
    p.Run(argc, argv);
    
    return 0; 
}