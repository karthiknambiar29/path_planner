#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

   /// Stop planning when mission control requests stop
   void setStop(const std_msgs::Bool::ConstPtr& stop);

   /// setOdom
   /// \param odom the current odometry pose
   void setOdom(const nav_msgs::Odometry::ConstPtr& odom);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan();

   /// Downsample the input occupancy grid for planning
   nav_msgs::OccupancyGrid::Ptr downsampleGrid(const nav_msgs::OccupancyGrid::Ptr& map, int factor);

   /// Convert world coordinates to grid coordinates
   void worldToGrid(double wx, double wy, float& gx, float& gy) const;

   /// Update start pose from odom -> base_link
   void updateStartFromOdom();

 private:
  /// The node handle
  ros::NodeHandle n;
  /// A publisher publishing the start position for RViz
  ros::Publisher pubStart;
  /// A subscriber for receiving map updates
  ros::Subscriber subMap;
  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;
   /// A subscriber for receiving odometry updates
   ros::Subscriber subOdom;
   /// A subscriber for receiving stop requests
   ros::Subscriber subStop;
   /// Publisher for no-path status
   ros::Publisher pubNoPath;
  /// The path produced by the hybrid A* algorithm
  Path path;
  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  Visualize visualization;
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
   /// Map resolution used to scale between world and grid coordinates
   float mapResolution = Constants::cellSize;
   /// Map origin in world coordinates
   double mapOriginX = 0.0;
   double mapOriginY = 0.0;
   /// Map downsampling factor (1 = no downsampling)
   int mapDownsampleFactor = 1;
  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start;
   /// Latest odom pose for start updates
   nav_msgs::Odometry::ConstPtr lastOdom;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
   /// Flag for stopping planning
   bool stopRequested = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
