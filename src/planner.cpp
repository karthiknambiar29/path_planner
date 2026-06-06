#include "planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  pubNoPath = n.advertise<std_msgs::Bool>("/path_planner/no_path", 1, true);
  std_msgs::Bool noPathMsg;
  noPathMsg.data = true;
  pubNoPath.publish(noPathMsg);

  // ___________________
  // TOPICS TO SUBSCRIBE
  const std::string mapTopic = "/occupancy_grid_node/occupancy_grid_downsampled";
  subMap = n.subscribe(mapTopic, 1, &Planner::setMap, this);

  n.param("map_downsample_factor", mapDownsampleFactor, 1);
  if (mapDownsampleFactor < 1) {
    mapDownsampleFactor = 1;
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
  subOdom = n.subscribe("/odom", 1, &Planner::setOdom, this);
  subStop = n.subscribe("/mission_control/stop", 1, &Planner::setStop, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
nav_msgs::OccupancyGrid::Ptr Planner::downsampleGrid(const nav_msgs::OccupancyGrid::Ptr& map, int factor) {
  if (factor <= 1) {
    return map;
  }

  nav_msgs::OccupancyGrid::Ptr downsampled(new nav_msgs::OccupancyGrid());
  downsampled->header = map->header;
  downsampled->info = map->info;
  downsampled->info.resolution = map->info.resolution * factor;
  downsampled->info.width = (map->info.width + factor - 1) / factor;
  downsampled->info.height = (map->info.height + factor - 1) / factor;
  downsampled->data.assign(downsampled->info.width * downsampled->info.height, 0);

  const int srcWidth = static_cast<int>(map->info.width);
  const int srcHeight = static_cast<int>(map->info.height);
  const int dstWidth = static_cast<int>(downsampled->info.width);
  const int dstHeight = static_cast<int>(downsampled->info.height);

  for (int y = 0; y < dstHeight; ++y) {
    const int y0 = y * factor;
    const int y1 = std::min(y0 + factor, srcHeight);
    for (int x = 0; x < dstWidth; ++x) {
      const int x0 = x * factor;
      const int x1 = std::min(x0 + factor, srcWidth);
      bool occupied = false;
      for (int yy = y0; yy < y1 && !occupied; ++yy) {
        const int row = yy * srcWidth;
        for (int xx = x0; xx < x1; ++xx) {
          const int8_t cell = map->data[row + xx];
          if (cell > 0) {
            occupied = true;
            break;
          }
        }
      }
      downsampled->data[y * dstWidth + x] = occupied ? 100 : 0;
    }
  }

  return downsampled;
}

void Planner::worldToGrid(double wx, double wy, float& gx, float& gy) const {
  gx = static_cast<float>((wx - mapOriginX) / mapResolution);
  gy = static_cast<float>((wy - mapOriginY) / mapResolution);
}

void Planner::updateStartFromOdom() {
  if (!lastOdom) {
    validStart = false;
    return;
  }

  start.pose.pose = lastOdom->pose.pose;
  const double yaw = tf::getYaw(start.pose.pose.orientation);
  const double forwardOffset = 1.7;
  start.pose.pose.position.x += forwardOffset * std::cos(yaw);
  start.pose.pose.position.y += forwardOffset * std::sin(yaw);

  float gridX = 0.0f;
  float gridY = 0.0f;
  worldToGrid(start.pose.pose.position.x, start.pose.pose.position.y, gridX, gridY);
  validStart = (grid->info.height >= gridY && gridY >= 0 && grid->info.width >= gridX && gridX >= 0);

  if (validStart) {
    geometry_msgs::PoseStamped startN;
    startN.pose.position = start.pose.pose.position;
    startN.pose.orientation = start.pose.pose.orientation;
    startN.header.frame_id = "odom";
    startN.header.stamp = ros::Time::now();
    pubStart.publish(startN);
  }
}

void Planner::setOdom(const nav_msgs::Odometry::ConstPtr& odom) {
  lastOdom = odom;
}

void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map; // downsampleGrid(map, mapDownsampleFactor);
  mapResolution = grid->info.resolution;
  mapOriginX = grid->info.origin.position.x;
  mapOriginY = grid->info.origin.position.y;
  path.setMapResolution(mapResolution);
  path.setMapOrigin(mapOriginX, mapOriginY);
  smoothedPath.setMapResolution(mapResolution);
  smoothedPath.setMapOrigin(mapOriginX, mapOriginY);
  visualization.setMapResolution(mapResolution);
  //update the configuration space with the current map
  configurationSpace.updateGrid(grid);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = grid->info.height;
  int width = grid->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const int8_t cell = grid->data[y * width + x];
      binMap[x][y] = (cell > 0);
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  updateStartFromOdom();
  if (validStart && validGoal) {
    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  (void)initial;
  updateStartFromOdom();
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  if (stopRequested) {
    return;
  }
  // retrieving goal position
  float x = 0.0f;
  float y = 0.0f;
  worldToGrid(end->pose.position.x, end->pose.position.y, x, y);
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::setStop(const std_msgs::Bool::ConstPtr& stop) {
  stopRequested = static_cast<bool>(stop->data);
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  if (stopRequested) {
    return;
  }
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    const int width = grid->info.width;
    const int height = grid->info.height;
    const int depth = Constants::headings;
    if (width <= 0 || height <= 0) {
      std::cout << "invalid map size, width: " << width << " height: " << height << std::endl;
      return;
    }

    const size_t widthSz = static_cast<size_t>(width);
    const size_t heightSz = static_cast<size_t>(height);
    const size_t depthSz = static_cast<size_t>(depth);
    if (widthSz > std::numeric_limits<size_t>::max() / heightSz ||
        (widthSz * heightSz) > std::numeric_limits<size_t>::max() / depthSz) {
      std::cout << "map size overflows allocation, width: " << width << " height: " << height
                << " headings: " << depth << std::endl;
      return;
    }

    const size_t nodes2DCount = widthSz * heightSz;
    const size_t nodes3DCount = nodes2DCount * depthSz;
    if (nodes3DCount > static_cast<size_t>(std::numeric_limits<int>::max())) {
      std::cout << "map too large for int indices, width: " << width << " height: " << height
                << " headings: " << depth << std::endl;
      return;
    }
    // define list pointers and initialize lists
    Node3D* nodes3D = new (std::nothrow) Node3D[nodes3DCount]();
    Node2D* nodes2D = new (std::nothrow) Node2D[nodes2DCount]();
    if (nodes3D == nullptr || nodes2D == nullptr) {
      const double mega = 1024.0 * 1024.0;
      const double bytes3D = static_cast<double>(nodes3DCount * sizeof(Node3D));
      const double bytes2D = static_cast<double>(nodes2DCount * sizeof(Node2D));
      std::cout << "failed to allocate planning grids (nodes3D: " << (bytes3D / mega)
                << " MB, nodes2D: " << (bytes2D / mega) << " MB)" << std::endl;
      delete [] nodes3D;
      delete [] nodes2D;
      return;
    }

    // ________________________
    // retrieving goal position
    float x = 0.0f;
    float y = 0.0f;
    worldToGrid(goal.pose.position.x, goal.pose.position.y, x, y);
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    worldToGrid(start.pose.pose.position.x, start.pose.pose.position.y, x, y);
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    std_msgs::Bool noPathMsg;
    noPathMsg.data = smoother.getPath().empty();
    pubNoPath.publish(noPathMsg);



    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
