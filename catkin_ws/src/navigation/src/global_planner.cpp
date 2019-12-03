#include <pluginlib/class_list_macros.h>
 #include "global_planner.h"
 #include "nav_msgs/Path.h"
 #include <iostream>
 #include <Python.h>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace global_planner {
  bool wait = true;
  std::vector<geometry_msgs::PoseStamped> path;
   void chatterCallback(const nav_msgs::Path& msg){
     std::cout << "JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ" << std::endl;
     path = msg.poses;
     wait = false;
   }

 GlobalPlanner::GlobalPlanner (){

 }

 GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

 }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

  //ros::init(0, nullptr, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("plan", 1000, chatterCallback);
  ros::Rate r(10);
  std::cout << "PRINT1" << std::endl;
  while(wait){
    r.sleep();
    std::cout << "~" << std::endl;
  }
  std::cout << "EXIT LOOP" << std::endl;
  for (geometry_msgs::PoseStamped& pose : path) {
    plan.push_back(pose);
  }
  wait = true;
  return true;
 }
 };