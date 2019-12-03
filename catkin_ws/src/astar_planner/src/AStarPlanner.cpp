#include "AStarPlanner.h"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <Python.h>
#include <pyhelper.hpp>
#include <stdio.h>
#include <conio.h>



//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_plugin::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_plugin
{

//Default Constructor
AStarPlanner::AStarPlanner()
{
}

/**
  Constructor that initilizes costmap and other parameters
**/
AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros);
}

/**
Implementation of method from BaseGlobalPlanner interface that
initializes the cost map and other parameters of the grid.

**/

void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{

}

/**
  Implementation of method from BaseGlobalPlanner interface that calculates
  plan to reach the goal
**/
bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan)
{
  CPyInstance pyInstance;
  char filename[] = "hello.py"
  FILE* fp;

  fp = Py_fopen(filename, "r");
  PyRun_SimpleFile(fp, filename);

}};