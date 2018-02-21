#include <ros/ros.h>
#include "TrafficLogger.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_traffic_logger") ;

  ros::NodeHandle nHandle ;
  
  TrafficLogger pioneerTrafficLogger(nHandle) ;
  
  ros::spin();
  return 0;
}
