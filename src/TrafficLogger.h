#include <ros/ros.h>
#include <ros/console.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

using std::vector ;

class TrafficLogger{
  public:
    TrafficLogger(ros::NodeHandle) ;
    ~TrafficLogger(){}
  private:
    ros::Subscriber subWaypointCmd ; // subscribe to all waypoint command times
    ros::Subscriber subGoalReached ; // subscribe to all move_base result messages from system traffic
    
    void goalReachedCallback(const move_base_msgs::MoveBaseActionResult&) ;
    void waypointCmdCallback(const geometry_msgs::PoseStamped&) ;
    vector<int> cmdTimes ;
    std::string robot_name ;
    std::string outputLogFile ;
} ;

TrafficLogger::TrafficLogger(ros::NodeHandle nh){
  // Read in robot name
  ros::param::get("pioneer_traffic_logger/robot_name",robot_name) ;
  
  // Initialise pub/sub nodes
  subGoalReached = nh.subscribe("move_base/result", 10, &TrafficLogger::goalReachedCallback, this) ;
  subWaypointCmd = nh.subscribe("goal_cmd_time", 10, &TrafficLogger::waypointCmdCallback, this) ;
  
  for (size_t i = 0; i < 4; i++)
    cmdTimes.push_back(0) ;
  
  // Get output log file name
  ros::param::get("pioneer_traffic_logger/output_logs",outputLogFile) ;
//  outputLogFile = "/home/jchu2041/catkin_ws/src/utm_traffic/log.csv" ;
}

void TrafficLogger::goalReachedCallback(const move_base_msgs::MoveBaseActionResult& msg){
  if (msg.status.status == 3){ // robot reached waypoint
    cmdTimes[2] = msg.header.stamp.sec ;
    cmdTimes[3] = msg.header.stamp.nsec ;
  }
  else{ // robot failed to reach waypoint
    cmdTimes[2] = -msg.header.stamp.sec ;
    cmdTimes[3] = -msg.header.stamp.nsec ;
  }
  
  // Filename to write to stored in A
  std::ofstream LogFile ;
  LogFile.open(outputLogFile.c_str(),std::ios::app) ;
  
  // Write in current cmdTimes
  LogFile << robot_name.c_str() << "," << cmdTimes[0] << "," << cmdTimes[1] << ","
    << cmdTimes[2] << "," << cmdTimes[3] << "\n" ;
  
  LogFile.close() ;
  printf("%s,%d,%d,%d,%d\n", robot_name.c_str(), cmdTimes[0], cmdTimes[1], cmdTimes[2], cmdTimes[3]) ;
  
  cmdTimes.clear() ;
  
  for (size_t i = 0; i < 4; i++)
    cmdTimes.push_back(0) ;
}

void TrafficLogger::waypointCmdCallback(const geometry_msgs::PoseStamped& msg){
  cmdTimes[0] = msg.header.stamp.sec ;
  cmdTimes[1] = msg.header.stamp.nsec ;
}
