#include <algorithm>
#include <string>
#include <memory>

#include <nav2_core/exceptions.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_bezier_controller/bezier_controller.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rmw/qos_profiles.h>
#include <visualization_msgs/msg/marker.hpp>


using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_bezier_controller
{
/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void BezierController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
      0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
      0.1));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  local_costmap_initialized_ = false;

  costmap_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/robot/local_costmap/costmap", 1, std::bind(&BezierController::costmapCallback, this, std::placeholders::_1));

  converter_subscriber_ = node->create_subscription<visualization_msgs::msg::Marker>("/robot/costmap_polygon_markers", 1, std::bind(&BezierController::converterCallback, this, std::placeholders::_1));

  global_publisher_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

  local_path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("received_local_plan", 1);
}

void BezierController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up BezierController.");
  global_publisher_.reset();
  local_path_publisher_.reset();
}

void BezierController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller BezierController");
  global_publisher_->on_activate();
  local_path_publisher_->on_activate();
}

void BezierController::deactivate()
{
  RCLCPP_INFO(logger_, "Dectivating controller BezierController");
  global_publisher_->on_deactivate();
  local_path_publisher_->on_deactivate();
}

void BezierController::setPlan(const nav_msgs::msg::Path & path)
{
  global_publisher_->publish(path);
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped BezierController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & , nav2_core::GoalChecker *)
{
  auto path_to_check = global_plan_;
  geometry_msgs::msg::TwistStamped cmd_vel;

  if(local_path.poses.size() > 0)
  {
    path_to_check = local_path;
  }

  if(isPlannedPathOccupied(path_to_check))
  {
    cmd_vel = followLocalPlan(pose);
  }
  else if(local_path.poses.size() > 0)
  {
    cmd_vel = followLocalPlan(pose);
  }
  else
  {
    cmd_vel = followGlobalPlan(pose);
  }

  return cmd_vel;
}

void BezierController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_INFO(logger_, "SpeedLimit: %f", speed_limit);
  RCLCPP_INFO(logger_, "In percentage?: %d", percentage);
}

bool BezierController::isPlannedPathOccupied(nav_msgs::msg::Path global_path)
{
  int path_size = global_path.poses.size();

  for(int i = 0; i < path_size; i++)
  {
    if(pointInLocalCostmap(global_path.poses[i]))
    {
      int cell_x, cell_y;
      worldToMap(global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y, cell_x, cell_y);

      if(isCellOccupied(cell_x, cell_y))
      {
        RCLCPP_INFO(logger_, "Path point (%f,%f) ist in der nähe eines Hindernis!", global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y);
        local_path = computeNewPath(global_plan_, global_path.poses[i]);
        return true;
      }
    }
    else
    {
      //Punkt liegt außerhalb der local Costmap
      return false;
    }
  }
  return false;
}

void BezierController::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  local_costmap_ = *msg;
  local_costmap_initialized_ = true;
}

void BezierController::converterCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  converter_marker_ = *msg;
}

bool BezierController::pointInLocalCostmap(const geometry_msgs::msg::PoseStamped path_point)
{
  bool isInLocalCostmap = true;

  double min_x = local_costmap_.info.origin.position.x;
  double max_x = local_costmap_.info.origin.position.x + (local_costmap_.info.width * local_costmap_.info.resolution);
  double min_y = local_costmap_.info.origin.position.y;
  double max_y = local_costmap_.info.origin.position.y + (local_costmap_.info.height * local_costmap_.info.resolution);

  if(path_point.pose.position.x < min_x || path_point.pose.position.x > max_x)
  {
    isInLocalCostmap = false;
  }
  else if(path_point.pose.position.y < min_y || path_point.pose.position.y > max_y)
  {
    isInLocalCostmap = false;
  }

  return isInLocalCostmap;
}

//Implementation von costmap_2d.cpp
void BezierController::worldToMap(double x, double y, int& cell_x, int& cell_y)
{
  //map resolution [m/cell]
  double resolution = local_costmap_.info.resolution;

  //info = nav_msgs/MapMetaData Message 
  //origin = real-world pose of the cell (0,0) in the map
  double origin_x = local_costmap_.info.origin.position.x;
  double origin_y = local_costmap_.info.origin.position.y;

  cell_x = abs(static_cast<int>((x - origin_x) / resolution));
  cell_y = abs(static_cast<int>((y - origin_y) / resolution));
}

bool BezierController::isCellOccupied(int cell_x, int cell_y, int threshold)
{
  if (!local_costmap_initialized_){
    //Costmap ist leer
    return false;
  }
  
  //getIndex von costmap_2d.cpp
  int index = cell_y * local_costmap_.info.width + cell_x;
  int occupancy = local_costmap_.data[index];

  return occupancy > threshold;
}

nav_msgs::msg::Path BezierController::computeNewPath(nav_msgs::msg::Path path, geometry_msgs::msg::PoseStamped occupied_path_point)
{
  RCLCPP_INFO(logger_, "Occupied path_point: (%f,%f)", occupied_path_point.pose.position.x, occupied_path_point.pose.position.y);

  //Nächster Punkt auf dem Pfad
  geometry_msgs::msg::PoseStamped start_pose = path.poses[0];

  int index = getLastPoseInsideCostmap(path);
  int path_size = path.poses.size();
  
  if(index + 20 < path_size-1)
  {
    index = index + 20;
  }

  geometry_msgs::msg::PoseStamped end_pose = path.poses[index];
  
  //Neuen Pfad initialisieren
  nav_msgs::msg::Path new_path;
  new_path.header.frame_id = path.header.frame_id;
  new_path.header.stamp = path.header.stamp;

  //Bezierkurve zwischen start_pose und end_pose berechnen
  std::vector<geometry_msgs::msg::PoseStamped> pFinal;

  auto controlPoint = computeControlPoint(start_pose, end_pose);

  for(double t = 0; t <= 1; t += 0.01)
  {
    pFinal = computeCubicBezier(start_pose.pose.position, controlPoint, end_pose.pose.position, t, pFinal);
  }

  new_path.poses.clear();

  for(geometry_msgs::msg::PoseStamped pose : pFinal)
  {
    pose.header.frame_id = path.header.frame_id;
    pose.header.stamp = path.header.stamp;
    new_path.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, "Computed new Path");

  return new_path;
}

int BezierController::getLastPoseInsideCostmap(nav_msgs::msg::Path path)
{
  int path_size = path.poses.size();
  int index = 0;

  for(int i = 0; i < path_size; ++i)
  {
    if(!pointInLocalCostmap(path.poses[i]))
    {
      index = i;
      break;
    }
  }

  return index;
}

geometry_msgs::msg::Point BezierController::computeControlPoint(geometry_msgs::msg::PoseStamped start_pose, geometry_msgs::msg::PoseStamped end_pose)
{
  geometry_msgs::msg::Point controlPoint;

  int size = converter_marker_.points.size();

  if(abs(start_pose.pose.position.x - end_pose.pose.position.x) > abs(start_pose.pose.position.y - end_pose.pose.position.y))
  {
    if(start_pose.pose.position.x < end_pose.pose.position.x)
    {
      //In positiver x-Richtung unterwegs
      
      //Rechts am Hindernis vorbei
      controlPoint.x = (start_pose.pose.position.x + end_pose.pose.position.x)/2;
      for(int i = 0; i < size; i++)
      { 
        //y-Koordinate des converter-markers rechts
        if(converter_marker_.points[i].y < start_pose.pose.position.y)
        {
          controlPoint.y = converter_marker_.points[i].y + 0.22; //robot radius
          break;
        }
      }

      //Links am Hindernis vorbei
    }
    else
    {
      //In negativer x-Richtung unterwegs
      
      //Rechts am Hindernis vorbei

      //Links am Hindernis vorbei
    }
  }
  else
  {
    if(start_pose.pose.position.y > end_pose.pose.position.y)
    {
      //In positiver y-Richtung unterwegs

      //Rechts am Hindernis vorbei
      
      //Links am Hindernis vorbei
      controlPoint.y = (start_pose.pose.position.y + end_pose.pose.position.y)/2;
      for(int i = 0; i < size; i++)
      { 
        //x-Koordinate des converter-markers links bzw. unten
        if(converter_marker_.points[i].x < start_pose.pose.position.x)
        {
          controlPoint.x = converter_marker_.points[i].x - 0.22; //robot radius
          break;
        }
      }
    }
    else
    {
      //In negativer y-Richtung unterwegs

      //Rechts am Hindernis vorbei

      //Links am Hindernis vorbei
    }
  }

  RCLCPP_INFO(logger_, "Control Point: (%f,%f)", controlPoint.x, controlPoint.y);

  return controlPoint;
}

std::vector<geometry_msgs::msg::PoseStamped> BezierController::computeCubicBezier(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point p3, double t, std::vector<geometry_msgs::msg::PoseStamped> pFinal)
{
  geometry_msgs::msg::PoseStamped newPose;
  
  newPose.pose.position.x = pow(1-t, 2) * p1.x + (1-t) * 2 * t * p2.x + t * t * p3.x;
  newPose.pose.position.y = pow(1-t, 2) * p1.y + (1-t) * 2 * t * p2.y + t * t * p3.y;

  pFinal.push_back(newPose);

  return pFinal;
}

geometry_msgs::msg::TwistStamped BezierController::followGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  //Das Stück des Pfads, welches innerhalb der local costmap liegt
  auto transformed_plan = transformGlobalPlan(pose);
  
  // Find the first pose which is at a distance greater than the specified lookahed distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
    });

  // If the last pose is still within lookahed distance, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;

  double linear_vel, angular_vel;

  // Compute the velocity using the pure pursuit algorithm
  auto curvature = 2.0 * goal_pose.position.y / (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
  linear_vel = desired_linear_vel_;
  angular_vel = desired_linear_vel_ * curvature;

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = max(
    -1.0 * abs(max_angular_vel_), min(
      angular_vel, abs(
        max_angular_vel_)));

  return cmd_vel;
}

geometry_msgs::msg::TwistStamped BezierController::followLocalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  auto transformed_plan = transformLocalPlan(pose);

  // Find the first pose which is at a distance greater than the specified lookahed distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
    });

  // If the last pose is still within lookahed distance, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;

  double linear_vel, angular_vel;

  // Compute the velocity using the pure pursuit algorithm
  auto curvature = 2.0 * goal_pose.position.y / (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
  linear_vel = desired_linear_vel_;
  angular_vel = desired_linear_vel_ * curvature;

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = max(
    -1.0 * abs(max_angular_vel_), min(
      angular_vel, abs(
        max_angular_vel_)));

  return cmd_vel;
}

//Implementation von nav2_dwb_controller
nav_msgs::msg::Path BezierController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(tf_, global_plan_.header.frame_id, pose,robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin = min_by(global_plan_.poses.begin(), global_plan_.poses.end(), [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
  });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(transformation_begin, end(global_plan_.poses), [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
  });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses), transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_publisher_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

nav_msgs::msg::Path BezierController::transformLocalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  if (local_path.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(tf_, local_path.header.frame_id, pose,robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global frame");
  }

  // First find the closest pose on the path to the robot
  auto transformation_begin = min_by(local_path.poses.begin(), local_path.poses.end(), [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
  });

  auto transformation_end = local_path.poses.end();

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses), transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  local_path.poses.erase(begin(local_path.poses), transformation_begin);
  
  if(local_path.poses.size() < 10)
  {
    local_path.poses.clear();
  }

  local_path_publisher_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool BezierController::transformPose(const std::shared_ptr<tf2_ros::Buffer> tf, const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose, const rclcpp::Duration & transform_tolerance)  
{
  if (in_pose.header.frame_id == frame) {

    out_pose = in_pose;
    return true;

  }

  try {

    tf->transform(in_pose, out_pose, frame);
    return true;
    
  } 
  catch (tf2::ExtrapolationException & ex) {

    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance)
    {

      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Transform data too old when converting from %s to %s", in_pose.header.frame_id.c_str(), frame.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Data time: %ds %uns, Transform time: %ds %uns", in_pose.header.stamp.sec, in_pose.header.stamp.nanosec, transform.header.stamp.sec, transform.header.stamp.nanosec);
      return false;

    } 
    else {

      tf2::doTransform(in_pose, out_pose, transform);
      return true;

    }
  } 
  catch (tf2::TransformException & ex) {

    RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in transformPose: %s", ex.what());
    return false;

  }

  return false;
}

}  // namespace nav2_bezier_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_bezier_controller::BezierController, nav2_core::Controller)