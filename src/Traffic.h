#include <ros/ros.h>
#include <ros/console.h>
#include <float.h>
#include <algorithm>
#include <iostream>
#include <string>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "agent_msgs/AgentMembership.h" // custom message for reporting parent (current) and child (next) agents
#include "agent_msgs/UtmGraph.h" // custom message for reporting link traversal costs over entire graph
#include "agent_msgs/BoolLog.h" // custom message including header and bool type
#include "agent_msgs/WallUpdate.h" // custom message defining all walls that must be added or removed
#include "Voronoi.h"

class Traffic
{
  public:
    Traffic(ros::NodeHandle) ;
    ~Traffic(){}
    
  private:
    // pub/sub nodes
    ros::Subscriber subAgentGraph ;
    ros::Subscriber subOdom ;
    ros::Subscriber subCmdVel ;
    ros::Subscriber subGoal ;
    ros::Publisher pubMembership ;
    ros::Publisher pubCmdVel ;
    ros::Publisher pubDelay ;
    ros::Publisher pubCostMapUpdate ;
    ros::Publisher pubMapGoal ;
    
    agent_msgs::AgentMembership membership ;
    agent_msgs::UtmGraph graph ;
    int numAgents ;
    int numVertices ;
    
    // link agents defined as source/sink voronoi pairs; source voronoi also defines current voronoi parition
    // note that agents[membership.parent][1] == agents[membership.child][0],
    // i.e. sink of parent link = source of child link
    vector< vector<int> > agents ;
    
    Voronoi cellMap ; // handles Voronoi partition membership operations
    int curV ; // current Voronoi parition, must equal agents[membership.parent][0] (initialised to -1)
    int goalV ; // Voronoi partition containing goal state
    vector<int> linkPath ; // list of link agents to goal
    vector<int> oldPath ; // most recent list of link agents for which virtual walls have been created
    
    bool firstGraph ; // First graph message has been received?
    bool graphLog ; // New graph message has been received since last Voronoi transit?
    bool cmdLog ; // First cmd_vel message has been received from move_base?
    bool goalLog ; // New map_goal message has been received?
    bool membershipAssigned ; // First membership assigned since receiving new goal?
    
    agent_msgs::BoolLog delayed ; // header+boolean defining if traffic is currently waiting to enter link
    geometry_msgs::Twist cmd ; // last received cmd_vel message from move_base, typically is rebroadcast directly, will be overwritten if traffic is delayed
    geometry_msgs::Twist goal ;
    
    // Callback functions
    void graphCallback(const agent_msgs::UtmGraph&) ;
    void odomCallback(const nav_msgs::Odometry&) ;
    void cmdVelCallback(const geometry_msgs::Twist&) ;
    void goalCallback(const geometry_msgs::Twist&) ;
    
    void ComputeHighPath(int, int) ; // Compute sequence of links to traverse
    void AdjustPath(bool &) ; // Replanning routine, output boolean determines if membership change occurred
    void UpdateCostMapLayer() ; // Publishes message to edit the virtual walls according to new path
    static bool ComparePQueue(vector<double>, vector<double>) ;
} ;

Traffic::Traffic(ros::NodeHandle nh): curV(-1), cmdLog(false), graphLog(false), firstGraph(false), membershipAssigned(false), goalLog(false){
  // Initialise pub/sub nodes
  subAgentGraph = nh.subscribe("/utm_graph", 10, &Traffic::graphCallback, this) ;
  subOdom = nh.subscribe("odom", 10, &Traffic::odomCallback, this) ;
  subCmdVel = nh.subscribe("recovery_cmd_vel", 10, &Traffic::cmdVelCallback, this) ;
  subGoal = nh.subscribe("cmd_map_goal", 10, &Traffic::goalCallback, this) ;
  pubMembership = nh.advertise<agent_msgs::AgentMembership>("membership", 10, true) ;
  pubCmdVel = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10, true) ;
  pubDelay = nh.advertise<agent_msgs::BoolLog>("delayed", 10) ;
  pubMapGoal = nh.advertise<geometry_msgs::Twist>("map_goal", 10) ;
  pubCostMapUpdate = nh.advertise<agent_msgs::WallUpdate>("move_base/global_costmap/blocking_layer/editWalls", 10, true) ;
  
  // Read in UTM parameters
  ros::param::get("/utm_agents/num_agents", numAgents);
  ros::param::get("/utm_agents/num_vertices", numVertices);
  
  char buffer[50] ;
  for (int i = 0; i < numAgents; i++){
    vector<int> temp(2,0) ;
    sprintf(buffer, "/utm_agents/agent%u/v0", i) ;
    ros::param::get(buffer, temp[0]);
    sprintf(buffer, "/utm_agents/agent%u/v1", i) ;
    ros::param::get(buffer, temp[1]);
    agents.push_back(temp) ;
    oldPath.push_back(i) ; // initialise old link path to include all agents (i.e. no walls initially present)
  }
  
  // Initialise log delay boolean
  delayed.data = false ;
  
  // Initialise membership robot name
  std::string robot_name ;
  ros::param::get("pioneer_traffic/robot_name",robot_name) ;
  membership.robot_name = robot_name ;
  
  ROS_INFO_STREAM("Robot name: " << membership.robot_name) ;
  
  ROS_INFO("***** Traffic initialisation complete! *****") ;
}

void Traffic::graphCallback(const agent_msgs::UtmGraph& msg){
  ROS_INFO("***** Graph callback *****") ;
  graph.actual_traversal_costs = msg.actual_traversal_costs ;
  graph.policy_output_costs = msg.policy_output_costs ;
  graph.wait_to_enter = msg.wait_to_enter ;
  graphLog = true ;
  if (!firstGraph)
    firstGraph = true ;
}

void Traffic::odomCallback(const nav_msgs::Odometry& msg){
  // I think this function is being called before something else
  // This would explain the inconsistency of this error (due to threading)
  if(membership.parent == -1)
    membershipAssigned = false;
  
//  ROS_INFO("***** Odom callback *****") ;
  double x = msg.pose.pose.position.x ;
  double y = msg.pose.pose.position.y ;
  
  delayed.header = msg.header ; // copy over header from odometry
  bool delay = delayed.data ;
  delayed.data = false ;

  
  // Compute current membership
  int newV = cellMap.Membership(x,y) ;
  if (newV >= 0)
    curV = newV ;
    
//  ROS_INFO_STREAM("Current position: (" << x << "," << y << "), Voronoi: " << curV ) ;
  
  // Plan from current position if new goal was received and graph was received
  if (goalLog && firstGraph){
    ROS_INFO("New goal was received...") ;
    ComputeHighPath(curV,goalV) ; // this will update linkPath
    UpdateCostMapLayer() ; // update virtual walls
    if (linkPath.size() > 0)
      membership.parent = linkPath[0] ;
    else{
      membership.parent = -1 ; // this should never be triggered here, i.e. no consecutive goals should be in same voronoi
      ROS_ERROR("Consecutive goals in same Voronoi cell");
    }
    
    if (linkPath.size() > 1)
      membership.child = linkPath[1] ;
    else
      membership.child = -1 ;
    
    ROS_INFO("Link path:") ;
    for (size_t i = 0; i < linkPath.size(); i++)
      ROS_INFO_STREAM(linkPath[i]) ;
    pubMembership.publish(membership) ; // announce membership change
    membershipAssigned = true ;
  }
  
  if (membershipAssigned){ // Only begin this routine after goal has been assigned
    if (membership.parent == -1){} // avoid superfluous transitions from odom errors on last leg
    else if (curV == agents[membership.parent][1]){ // Determine if Voronoi transit occurred
      ROS_INFO("Transit detected...") ;
      // Update memberships
      linkPath.erase(linkPath.begin()) ; // remove top link from path 
      if (linkPath.size() > 0)
        membership.parent = linkPath[0] ;
      else
        membership.parent = -1 ; // in final voronoi, curV == goalV
      
      if (linkPath.size() > 1)
        membership.child = linkPath[1] ;
      else
        membership.child = -1 ; // on final link
      
      // Allow replanning from parent node sink voronoi if new graph was received during last transit
      if (graphLog && membership.child >= 0){
        bool diff ;
        AdjustPath(diff) ;
      }
      ROS_INFO_STREAM("New membership.parent: " << membership.parent << ", membership.child: " << membership.child) ;
      pubMembership.publish(membership) ; // announce membership change
    }
    else { // No voronoi transition detected, determine if currently adjacent to next voronoi
      bool adj = cellMap.MovingToNextVoronoi(x,y,agents[membership.parent][1]) ;
      if (membership.child >= 0 && membership.child < numAgents){
        if (adj && graph.wait_to_enter[membership.child]){
          if (!delay)
            ROS_INFO("Must wait to enter adjacent link...") ;
          
          // Adjacent to next Voronoi, must wait to enter
          delayed.data = true ;
          
          // Stop platform
          cmd.linear.x = 0.0 ;
          cmd.linear.y = 0.0 ;
          cmd.linear.z = 0.0 ;
          cmd.angular.x = 0.0 ;
          cmd.angular.y = 0.0 ;
          cmd.angular.z = 0.0 ;
          
          // Allow replanning from parent node sink voronoi if new graph was received since last planning time
          if (graphLog){
            bool diff ;
            AdjustPath(diff) ;
            if (diff)
              pubMembership.publish(membership) ;
          }
        }
      }
    }
  
    if (goalLog){
      // Publish goal to move_base once processed
      pubMapGoal.publish(goal) ;
      goalLog = false ; // current goal has been processed
    }
    pubDelay.publish(delayed) ; // published for rosbag logging
    
  }
  pubCmdVel.publish(cmd) ;
}

void Traffic::cmdVelCallback(const geometry_msgs::Twist& msg){
//  ROS_INFO("***** CmdVel callback *****") ;
  cmd.linear.x = msg.linear.x ;
  cmd.linear.y = msg.linear.y ;
  cmd.linear.z = msg.linear.z ;
  cmd.angular.x = msg.angular.x ;
  cmd.angular.y = msg.angular.y ;
  cmd.angular.z = msg.angular.z ;
  cmdLog = true ;
}

void Traffic::goalCallback(const geometry_msgs::Twist& msg){
  ROS_INFO("***** Goal callback *****") ;
  goal.linear.x = msg.linear.x ;
  goal.linear.y = msg.linear.y ;
  goal.linear.z = msg.linear.z ;
  goal.angular.x = msg.angular.x ;
  goal.angular.y = msg.angular.y ;
  goal.angular.z = msg.angular.z ;
  goalLog = true ;
  
  goalV = cellMap.Membership(goal.linear.x,goal.linear.y) ;
  ROS_INFO_STREAM("New goal commanded at (" << goal.linear.x << "," << goal.linear.y << "), Sector: " << goalV) ;
}

void Traffic::ComputeHighPath(int s, int g){
  ROS_INFO("Computing high level path...") ;
  
  // Using Dijkstra's, only store best parent vertex ID
  vector<int> closed(numVertices,-1) ;
  
  vector< vector<double> > pQueue ;
  for (size_t i = 0; i < numVertices; i++){
    vector<double> e ;
    e.push_back((double)i) ; // vertex ID
    e.push_back(-1.0) ; // best parent vertex ID
    e.push_back(DBL_MAX) ; // cost to arrive
    pQueue.push_back(e) ;
  }
  
  pQueue[s][2] = 0.0 ; // set cost to arrive for start vertex
  bool gReached = false ;
  while (!gReached && !pQueue.empty()){
    std::sort(pQueue.begin(),pQueue.end(),ComparePQueue) ;
    
    // Pop the first element
    vector<double> v = pQueue[0] ;
    pQueue.erase(pQueue.begin()) ;
    
    // Log best parent into closed set
    closed[(int)v[0]] = (int)v[1] ;
    
    // Check if goal is reached
    if ((int)v[0] == g){
      gReached = true ;
      break ;
    }
    
    // Find all children
    vector<int> c ;
    vector<double> d ;
    for (size_t i = 0; i < agents.size(); i++){
      if (agents[i][0] == (int)v[0]){
        c.push_back(agents[i][1]) ;
        // compute cost to arrive from v
        double c2a = v[2] + graph.actual_traversal_costs[i] + graph.policy_output_costs[i] ;
        d.push_back(c2a) ;
      }
    }
    
    // Update pQueue cost to arrive for all children
    for (size_t i = 0; i < c.size(); i++){
      for (size_t j = 0; j < pQueue.size(); j++){
        if ((int)pQueue[j][0] == c[i]){
          if (pQueue[j][2] > d[i]){
            pQueue[j][1] = v[0] ; // change best parent
            pQueue[j][2] = d[i] ; // change cost to arrive
          }
          break ;
        }
      }
    }
  }
  
  if (!gReached)
    ROS_ERROR("[Traffic::ComputeHighpath()]: no path found!") ;
  else if(s == g){
    ROS_INFO("Start equals goal");
    return;
  } else {
    int v = g ;
    vector<int> rPath ; // path from goal to source
    while (v != s){
      int p = closed[v] ;
      int ind ;
      for (size_t i = 0; i < agents.size(); i++){
        if (agents[i][0] == p && agents[i][1] == v){
          ind = i ;
          break ;
        }
      }
      rPath.push_back(ind) ;
      v = p ;
    }
    
    linkPath.clear() ;
    for (size_t i = 0; i < rPath.size(); i++){
      size_t j = rPath.size() - i - 1 ;
      linkPath.push_back(rPath[j]) ;
    }
  }
  graphLog = false ; // latest graph has been accounted for
  ROS_INFO("High level path calculated...") ;
}

void Traffic::AdjustPath(bool & diff){
  ROS_INFO("Replanning path...") ;
  ComputeHighPath(agents[membership.parent][1],goalV) ; // this will update linkPath
  
  // Determine if new path is different to old path, note that new linkPath does not contain parent link
  diff = false ;
  for (size_t i = 0; i < linkPath.size(); i++){
    if (linkPath[i] != oldPath[i+1]){
      diff = true ;
      break ;
    }
  }
  
  // Adjust values if new path is different
  if (diff){
    // Replace parent in link path since transition still pending
    vector<int> temp = linkPath ;
    linkPath.clear() ;
    linkPath.push_back(membership.parent) ;
    for (size_t i = 0; i < temp.size(); i++)
      linkPath.push_back(temp[i]) ;
    
    if (linkPath.size() > 1){
      if (membership.child == linkPath[1])
        diff = false ; // no need to publish membership message since no change
      else
        membership.child = linkPath[1] ;
    }
    else
      membership.child = -1 ; // on final link, this should never be triggered here
    
    UpdateCostMapLayer() ;
  }
  else { // retain old path if no change detected
    linkPath.clear() ;
    linkPath = oldPath ;
  }
}

void Traffic::UpdateCostMapLayer(){
  ROS_INFO("Updating virtual walls...") ;
  agent_msgs::WallUpdate update ;
  update.total_changes = 0 ;
  
  // Check for walls that must be removed (i.e. in linkPath but not in oldPath)
  // Check for walls that must be added (i.e. in oldPath but not in linkPath)
  // Note that agents i and i+1, where i/2 == i+1/2, share the same wall
  vector<int> toRemove ;
  vector<int> toAdd ;
  vector<bool> found(oldPath.size(),false) ;
//  ROS_INFO_STREAM("oldPath.size(): " << oldPath.size()) ;
//  for (size_t j = 0; j < oldPath.size(); j++)
//    ROS_INFO_STREAM(oldPath[j]) ;
//  
//  ROS_INFO_STREAM("linkPath.size(): " << linkPath.size()) ;
//  for (size_t j = 0; j < linkPath.size(); j++)
//    ROS_INFO_STREAM(linkPath[j]) ;
  
  for (size_t i = 0; i < linkPath.size(); i++){
    bool newLink = true ;
    for (size_t j = 0; j < oldPath.size(); j++){
      if (linkPath[i]/2 == oldPath[j]/2){
        newLink = false ;
        found[j] = true ;
//        ROS_INFO_STREAM("link: " << oldPath[j] << " on old path will be removed") ;
      }
    }
    if (newLink)
      toRemove.push_back(linkPath[i]) ;
  }
  for (size_t i = 0; i < found.size(); i++)
    if (!found[i])
      toAdd.push_back(oldPath[i]) ;
  
//  ROS_INFO_STREAM("toRemove.size(): " << toRemove.size()) ;
//  for (size_t i = 0; i < toRemove.size(); i++)
//    ROS_INFO_STREAM(toRemove[i]) ;
//  ROS_INFO_STREAM("toAdd.size(): " << toAdd.size()) ;
//  for (size_t i = 0; i < toAdd.size(); i++)
//    ROS_INFO_STREAM(toAdd[i]) ;
  
  char buffer[50] ;
  for (int i = 0; i < toRemove.size(); i++){
    int numWalls ;
    sprintf(buffer, "/utm_agents/agent%u/num_walls", toRemove[i]) ;
    ros::param::get(buffer, numWalls);
    for (int j = 0; j < numWalls; j++){
      vector<float> wall ;
      sprintf(buffer, "/utm_agents/agent%u/wall%u", toRemove[i], j) ;
      ros::param::get(buffer, wall) ;
      update.total_changes++ ;
      update.type.push_back(false) ; // removing wall
      for (size_t k = 0; k < wall.size(); k++)
        update.data.push_back(wall[k]) ;
    }
  }
   
  for (int i = 0; i < toAdd.size(); i++){
    int numWalls ;
    sprintf(buffer, "/utm_agents/agent%u/num_walls", toAdd[i]) ;
    ros::param::get(buffer, numWalls);
    for (int j = 0; j < numWalls; j++){
      vector<float> wall ;
      sprintf(buffer, "/utm_agents/agent%u/wall%u", toAdd[i], j) ;
      ros::param::get(buffer, wall) ;
      update.total_changes++ ;
      update.type.push_back(true) ; // adding wall
      for (size_t k = 0; k < wall.size(); k++)
        update.data.push_back(wall[k]) ;
    }
  }   
  
  oldPath.clear() ;
  oldPath = linkPath ;
  
  ROS_INFO("Virtual walls published...") ;
  pubCostMapUpdate.publish(update) ;
}

bool Traffic::ComparePQueue(vector<double> A, vector<double> B){
  return (A[2] < B[2]) ;
}
