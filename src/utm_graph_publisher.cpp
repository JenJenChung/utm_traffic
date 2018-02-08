#include <ros/ros.h>
#include <vector>
#include "agent_msgs/UtmGraph.h"

using std::vector ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_graph_publisher");

  ros::NodeHandle nHandpub;
  ros::Publisher pub = nHandpub.advertise<agent_msgs::UtmGraph>("/utm_graph", 10, true);
  
  agent_msgs::UtmGraph msg ;
  
  for (size_t i = 0; i < 38; i++){
    if (i == -1){
      msg.actual_traversal_costs.push_back(0.0) ;
      msg.policy_output_costs.push_back(0.0) ;
    }
    else{
      msg.actual_traversal_costs.push_back(0.0) ;
      msg.policy_output_costs.push_back(0.0) ;
    }
    if (i == -1)
      msg.wait_to_enter.push_back(true) ;
    else
      msg.wait_to_enter.push_back(false) ;
  }
  
  pub.publish(msg) ;

  ros::spin();
  return 0;
}
