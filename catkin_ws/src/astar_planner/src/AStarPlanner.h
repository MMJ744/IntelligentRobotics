
#include <string.h>
#include <conio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <set>
#include <Python.h>

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;
using std::string;

#ifndef ASTAR_PLANNER_CPP
#define ASTAR_PLANNER_CPP


namespace astar_plugin
{

class AStarPlanner : public nav_core::BaseGlobalPlanner
{
public:
  AStarPlanner();
  AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  
  /** overriden methods from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);
};
};
#endif