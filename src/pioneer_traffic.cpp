#include <ros/ros.h>
#include "Traffic.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_traffic") ;

  ros::NodeHandle nHandle ;
  
  Traffic pioneerTraffic(nHandle) ;
  
  ros::spin();
  return 0;
}
