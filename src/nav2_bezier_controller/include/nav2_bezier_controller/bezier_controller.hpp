#ifndef NAV2_BEZIER_CONTROLLER__BEZIER_CONTROLLER_HPP_
#define NAV2_BEZIER_CONTROLLER__BEZIER_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_bezier_controller
{

class BezierController : public nav2_core::Controller
{
public:
  BezierController() = default;
  ~BezierController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity /* velocity */, nav2_core::GoalChecker * /* goal_checker */) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  bool isPlannedPathOccupied(nav_msgs::msg::Path global_path);
  
  geometry_msgs::msg::TwistStamped followGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  
  geometry_msgs::msg::TwistStamped followLocalPlan(const geometry_msgs::msg::PoseStamped & pose);
  
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
	
  nav_msgs::msg::Path transformLocalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance
  );

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  
  void converterCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
  
  bool isCellOccupied(int cell_x, int cell_y, int threshold = 98);

  void worldToMap(double x, double y, int& cell_x, int& cell_y);
	
  bool pointInLocalCostmap(const geometry_msgs::msg::PoseStamped path_point);
  
  nav_msgs::msg::Path computeNewPath(nav_msgs::msg::Path path, const geometry_msgs::msg::PoseStamped occupied_path_point);
  
  int getLastPoseInsideCostmap(nav_msgs::msg::Path path);
  
  geometry_msgs::msg::Point computeControlPoint(geometry_msgs::msg::PoseStamped start_pose, geometry_msgs::msg::PoseStamped end_pose);
  
  std::vector<geometry_msgs::msg::PoseStamped> computeCubicBezier(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point p3, double t, std::vector<geometry_msgs::msg::PoseStamped> pFinal);
  
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("BezierController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  rclcpp::Duration transform_tolerance_ {0, 0};

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_path_publisher_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr converter_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  nav_msgs::msg::OccupancyGrid local_costmap_;
  bool local_costmap_initialized_;
  nav_msgs::msg::Path local_path;
  visualization_msgs::msg::Marker converter_marker_;
};

}  // namespace nav2_bezier_controller

#endif  // NAV2_BEZIER_CONTROLLER__BEZIER_CONTROLLER_HPP_
