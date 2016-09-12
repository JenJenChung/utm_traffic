#include <ros/ros.h>
#include "Traffic_sim.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_traffic_sim") ;

  ros::NodeHandle nHandle ;
  
  Traffic pioneerTraffic(nHandle) ;
  
  ros::spin();
  return 0;
}
