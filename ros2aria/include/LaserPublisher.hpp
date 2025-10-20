#ifndef ROSARIA_LASER_PUBLISHER_HPP
#define ROSARIA_LASER_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>

class ArLaser;
class ArTime;

class LaserPublisher
{
public:
  LaserPublisher(
    ArLaser* laser,
    rclcpp::Node::SharedPtr node,
    bool broadcast_transform = true,
    const std::string& tf_frame = "laser",
    const std::string& parent_tf_frame = "base_link",
    const std::string& global_tf_frame = "odom");

  ~LaserPublisher();

private:
  ArFunctorC<LaserPublisher> laserReadingsCB;
  
protected:
  void readingsCallback();
  void publishLaserScan();
  void publishPointCloud();

  rclcpp::Node::SharedPtr node_;
  ArLaser* laser_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub_;

  sensor_msgs::msg::LaserScan laserscan_;
  sensor_msgs::msg::PointCloud pointcloud_;

  std::string tf_frame_;
  std::string parent_tf_frame_;
  std::string global_tf_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool broadcast_tf_;
};

#endif  // ROSARIA_LASER_PUBLISHER_HPP
