#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include "LaserPublisher.hpp"
#include "ArTimeToROSTime.hpp"

#include <cmath>
#include <boost/algorithm/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

LaserPublisher::LaserPublisher(
  ArLaser* laser,
  rclcpp::Node::SharedPtr node,
  bool broadcast_tf,
  const std::string& tf_frame,
  const std::string& parent_tf_frame,
  const std::string& global_tf_frame)
  : node_(node),
    laser_(laser),
    tf_frame_(tf_frame),
    parent_tf_frame_(parent_tf_frame),
    global_tf_frame_(global_tf_frame),
    broadcast_tf_(broadcast_tf),
    laserReadingsCB(this, &LaserPublisher::readingsCallback) 
{
  if (!laser_) {
    RCLCPP_ERROR(node_->get_logger(), "LaserPublisher: null ArLaser pointer.");
    return;
  }

  laser_->lockDevice();
  laser_->addReadingCB(&laserReadingsCB);
  laser_->unlockDevice();
  // ...existing code...


  std::string laserscan_name(laser_->getName());
  boost::erase_all(laserscan_name, ".");
  laserscan_name += "_laserscan";

  std::string pointcloud_name(laser_->getName());
  boost::erase_all(pointcloud_name, ".");
  pointcloud_name += "_pointcloud";

  laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_name, 20);
  pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>(pointcloud_name, 50);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  tf2::Quaternion q;
  geometry_msgs::msg::TransformStamped transform_msg;

  if (laser_->hasSensorPosition()) {
    transform_msg.transform.translation.x = laser_->getSensorPositionX() / 1000.0;
    transform_msg.transform.translation.y = laser_->getSensorPositionY() / 1000.0;
    transform_msg.transform.translation.z = laser_->getSensorPositionZ() / 1000.0;
    q.setRPY(0, 0, ArMath::degToRad(laser_->getSensorPositionTh()));
  } else {
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = 0.0;
    q.setRPY(0, 0, 0);
  }

  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  laserscan_.header.frame_id = tf_frame_;
  laserscan_.angle_min = ArMath::degToRad(laser_->getStartDegrees());
  laserscan_.angle_max = ArMath::degToRad(laser_->getEndDegrees());
  laserscan_.range_min = 0.0;
  laserscan_.range_max = laser_->getMaxRange() / 1000.0;
  pointcloud_.header.frame_id = global_tf_frame_;

  laserscan_.angle_increment = 0.0;
  if (laser_->canSetIncrement()) {
    laserscan_.angle_increment = laser_->getIncrement();
  } else if (laser_->getIncrementChoice() != nullptr) {
    laserscan_.angle_increment = laser_->getIncrementChoiceDouble();
  }
  laserscan_.angle_increment *= M_PI / 180.0;
}

LaserPublisher::~LaserPublisher()
{
  if (laser_) {
    laser_->lockDevice();
    laser_->remReadingCB(&laserReadingsCB);
    laser_->unlockDevice();
  }
}

void LaserPublisher::readingsCallback()
{
  if (!laser_) return;

  laser_->lockDevice();
  publishLaserScan();
  publishPointCloud();
  laser_->unlockDevice();

  if (broadcast_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = convertArTimeToROS2(laser_->getLastReadingTime());
    tf_msg.header.frame_id = parent_tf_frame_;
    tf_msg.child_frame_id = tf_frame_;

    tf_msg.transform.translation.x = laser_->getSensorPositionX() / 1000.0;
    tf_msg.transform.translation.y = laser_->getSensorPositionY() / 1000.0;
    tf_msg.transform.translation.z = laser_->getSensorPositionZ() / 1000.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, ArMath::degToRad(laser_->getSensorPositionTh()));
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void LaserPublisher::publishLaserScan()
{
  laserscan_.header.stamp = convertArTimeToROS2(laser_->getLastReadingTime());

  const std::list<ArSensorReading*>* readings = laser_->getRawReadings();
  if (!readings) return;

  laserscan_.ranges.resize(readings->size());
  size_t n = 0;

  if (laser_->getFlipped()) {
    for (auto r = readings->rbegin(); r != readings->rend(); ++r) {
      laserscan_.ranges[n++] = (*r && !(*r)->getIgnoreThisReading()) ? (*r)->getRange() / 1000.0 : -1.0;
    }
  } else {
    for (auto r = readings->begin(); r != readings->end(); ++r) {
      laserscan_.ranges[n++] = (*r && !(*r)->getIgnoreThisReading()) ? (*r)->getRange() / 1000.0 : -1.0;
    }
  }

  laserscan_pub_->publish(laserscan_);
}

void LaserPublisher::publishPointCloud()
{
  if (!laser_) return;

  pointcloud_.header.stamp = convertArTimeToROS2(laser_->getLastReadingTime());

  const std::list<ArPoseWithTime*>* buffer = laser_->getCurrentRangeBuffer()->getBuffer();
  if (!buffer) return;

  pointcloud_.points.resize(buffer->size());
  size_t n = 0;

  for (auto i = buffer->cbegin(); i != buffer->cend(); ++i) {
    pointcloud_.points[n].x = (*i)->getX() / 1000.0;
    pointcloud_.points[n].y = (*i)->getY() / 1000.0;
    pointcloud_.points[n].z = (laser_->hasSensorPosition() ? laser_->getSensorPositionZ() / 1000.0 : 0.0);
    ++n;
  }

  pointcloud_pub_->publish(pointcloud_);
}
