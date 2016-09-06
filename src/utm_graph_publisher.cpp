#include <ros/ros.h>
#include <vector>
#include "agent_msgs/UtmGraph.h"

using std::vector ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_graph_publisher");

  ros::NodeHandle nHandpub;
  ros::Publisher pub = nHandpub.advertise<agent_msgs::UtmGraph>("/pioneer1/utm_graph", 10, true);
  
  agent_msgs::UtmGraph msg ;
  
  for (size_t i = 0; i < 10; i++){
    if (i == -1){
      msg.actual_traversal_costs.push_back(10.0) ;
      msg.policy_output_costs.push_back(100.0) ;
    }
    else{
      msg.actual_traversal_costs.push_back(10.0) ;
      msg.policy_output_costs.push_back(10.0) ;
    }
    if (i == 5)
      msg.wait_to_enter.push_back(true) ;
    else
      msg.wait_to_enter.push_back(false) ;
  }
  
  pub.publish(msg) ;

  ros::spin();
  return 0;
}
