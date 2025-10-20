// ros2_rosaria_node.cpp
// ROS 2 (Jazzy) port of RosAria (single-file example)
// Build: add to ament_cmake package, link against Aria and ROS2 libraries

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"    


#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "ros2aria_interfaces/msg/bumper_state.hpp" // custom message for bumpers

#include <Aria.h>
#include <ArRobotConfigPacketReader.h> // as in original

using std::placeholders::_1;

class RosAriaNode : public rclcpp::Node
{
public:
  explicit RosAriaNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ros2aria", options),
    conn(nullptr),
    laserConnector(nullptr),
    robot(nullptr),
    TicksMM(-1),
    DriftFactor(-99999),
    RevCount(-1),
    sonar_enabled(false),
    publish_aria_lasers(false),
    published_motors_state(false),
    cmdvel_timeout(rclcpp::Duration::from_seconds(0.6)) 

  {
    // declare parameters and load defaults
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baud", 0);
    declare_parameter<bool>("debug_aria", false);
    declare_parameter<std::string>("aria_log_filename", "Aria.log");
    declare_parameter<bool>("publish_aria_lasers", false);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_link_frame", "base_link");
    declare_parameter<std::string>("bumpers_frame", "bumpers");
    declare_parameter<std::string>("sonar_frame", "sonar");
    declare_parameter<double>("cmd_vel_timeout", 0.6);
    declare_parameter<int>("TicksMM", -1);
    declare_parameter<int>("DriftFactor", -99999);
    declare_parameter<int>("RevCount", -1);

    // read parameters
    this->get_parameter("port", serial_port);
    this->get_parameter("baud", serial_baud);
    this->get_parameter("debug_aria", debug_aria);
    this->get_parameter("aria_log_filename", aria_log_filename);
    this->get_parameter("publish_aria_lasers", publish_aria_lasers);
    this->get_parameter("odom_frame", frame_id_odom);
    this->get_parameter("base_link_frame", frame_id_base_link);
    this->get_parameter("bumpers_frame", frame_id_bumper);
    this->get_parameter("sonar_frame", frame_id_sonar);
    this->get_parameter("TicksMM", TicksMM);
    this->get_parameter("DriftFactor", DriftFactor);
    this->get_parameter("RevCount", RevCount);

    RCLCPP_INFO(get_logger(), "RosAria: set port: [%s]", serial_port.c_str());
    if (serial_baud != 0) {
      RCLCPP_INFO(get_logger(), "RosAria: set serial port baud rate %d", serial_baud);
    }

    // Publishers
    pose_pub = create_publisher<nav_msgs::msg::Odometry>("pose", 10);
//    bumpers_pub = create_publisher<sensor_msgs::msg::PointCloud>("bumper_state", 10); // using PointCloud only as placeholder for custom msg
    bumpers_pub = create_publisher<ros2aria_interfaces::msg::BumperState>("bumper_state", 10); // using PointCloud only as placeholder for custom msg
    
    sonar_pub = create_publisher<sensor_msgs::msg::PointCloud>("sonar", 50);
    sonar_pointcloud2_pub = create_publisher<sensor_msgs::msg::PointCloud2>("sonar_pointcloud2", 50);
    voltage_pub = create_publisher<std_msgs::msg::Float64>("battery_voltage", 10);
    recharge_state_pub = create_publisher<std_msgs::msg::Int8>("battery_recharge_state", 5);
    state_of_charge_pub = create_publisher<std_msgs::msg::Float32>("battery_state_of_charge", 10);
    motors_state_pub = create_publisher<std_msgs::msg::Bool>("motors_state", 5);

    // Subscriber (cmd_vel)
    cmdvel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&RosAriaNode::cmdvel_cb, this, _1)
    );

    // Services
    enable_srv = create_service<std_srvs::srv::Empty>(
      "enable_motors", std::bind(&RosAriaNode::enable_motors_cb, this, std::placeholders::_1, std::placeholders::_2)
    );
    disable_srv = create_service<std_srvs::srv::Empty>(
      "disable_motors", std::bind(&RosAriaNode::disable_motors_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    // parameter change callback (runtime config)
    param_cb_handle = add_on_set_parameters_callback(std::bind(&RosAriaNode::on_parameter_event, this, _1));

    // watchdog timer for cmd_vel
    double cmdvel_timeout_param = 0.6;
    this->get_parameter("cmd_vel_timeout", cmdvel_timeout_param);
    cmdvel_timeout = rclcpp::Duration::from_seconds(cmdvel_timeout_param);
    if (cmdvel_timeout_param > 0.0) {
      cmdvel_watchdog_timer = create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&RosAriaNode::cmdvel_watchdog, this));
    }

    // TF broadcaster
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ARIA init and setup
    Aria::init();
    if (Setup() != 0) {
      RCLCPP_FATAL(get_logger(), "RosAria: Setup failed");
      throw std::runtime_error("RosAria setup failed");
    }

    veltime = now();
  }

  ~RosAriaNode() override
  {
    if (robot) {
      robot->disableMotors();
      robot->disableSonar();
      robot->stopRunning();
      robot->waitForRunExit();
    }
    Aria::shutdown();
  }

  int Setup()
  {
    robot = new ArRobot();
    ArArgumentBuilder *args = new ArArgumentBuilder();
    ArArgumentParser *argparser = new ArArgumentParser(args);
    argparser->loadDefaultArguments();

    // parse serial port (hostname:port) or tty
    size_t colon_pos = serial_port.find(":");
    if (colon_pos != std::string::npos) {
      args->add("-remoteHost");
      args->add(serial_port.substr(0, colon_pos).c_str());
      args->add("-remoteRobotTcpPort");
      args->add(serial_port.substr(colon_pos+1).c_str());
    } else {
      args->add("-robotPort %s", serial_port.c_str());
    }

    if (serial_baud != 0) {
      args->add("-robotBaud %d", serial_baud);
    }

    if (debug_aria) {
      args->add("-robotLogPacketsReceived");
      args->add("-robotLogPacketsSent");
      args->add("-robotLogVelocitiesReceived");
      args->add("-robotLogMovementSent");
      args->add("-robotLogMovementReceived");
      ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
    }

    conn = new ArRobotConnector(argparser, robot);
    if (!conn->connectRobot()) {
      RCLCPP_ERROR(get_logger(), "RosAria: ARIA could not connect to robot!");
      return 1;
    }

    if (publish_aria_lasers) {
      laserConnector = new ArLaserConnector(argparser, robot, conn);
    }

    if(!Aria::parseArgs()) {
      RCLCPP_ERROR(get_logger(), "RosAria: ARIA error parsing ARIA startup parameters!");
      return 1;
    }

    // Lock while setting robot parameters (if provided)
    robot->lock();
    if (TicksMM > 0) robot->comInt(93, TicksMM);
    if (DriftFactor != -99999) robot->comInt(89, DriftFactor);
    if (RevCount > 0) robot->comInt(88, RevCount);
    robot->unlock();

    // enable motors and disable sonar initially (match ROS1 behavior)
    robot->enableMotors();
    robot->disableSonar();

    // Add sensor interp task callback (same idea as before)
    // We'll schedule publish() to be called by ARIA thread
    publishCB = new ArFunctorC<RosAriaNode>(this, &RosAriaNode::publish);
    robot->addSensorInterpTask("ROSPublishingTask", 100, publishCB);

    // Initialize bumpers size if you use the same bumper message type as in ROS1
    // (Here we don't have the custom rosaria::BumperState message; you'll likely
    // provide an equivalent or keep a custom message package in ROS2)

    robot->runAsync(true);

    if (publish_aria_lasers && laserConnector) {
      RCLCPP_INFO(get_logger(), "rosaria: connecting lasers...");
      if (!laserConnector->connectLasers()) {
        RCLCPP_FATAL(get_logger(), "rosaria: Error connecting to laser(s)...");
        return 1;
      }
      // create LaserPublisher nodes or objects similarly (not implemented here)
    }

    RCLCPP_INFO(get_logger(), "rosaria: Setup complete");
    return 0;
  }

private:
  // ------------ member variables ------------
  // ROS2 objects
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<ros2aria_interfaces::msg::BumperState>::SharedPtr bumpers_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr sonar_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pointcloud2_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_of_charge_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_srv;

  rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle;

  rclcpp::Time veltime;
  rclcpp::Duration cmdvel_timeout;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // ARIA objects
  ArRobotConnector *conn;
  ArLaserConnector *laserConnector;
  ArRobot *robot;
  ArFunctorC<RosAriaNode> *publishCB;

  // parameters
  std::string serial_port;
  int serial_baud;
  bool debug_aria;
  std::string aria_log_filename;
  bool publish_aria_lasers;

  std::string frame_id_odom;
  std::string frame_id_base_link;
  std::string frame_id_bumper;
  std::string frame_id_sonar;

  // odom msg storage
  nav_msgs::msg::Odometry position;
  ros2aria_interfaces::msg::BumperState bumpers;

  // robot calibration params
  int TicksMM;
  int DriftFactor;
  int RevCount;

  // sonar / state flags
  bool sonar_enabled;
  bool publish_sonar;
  bool published_motors_state;

  // bookkeeping messages
  std_msgs::msg::Int8 recharge_state;
  std_msgs::msg::Bool motors_state;

  // ------------ methods ------------

  // parameter callback (runtime changes)
  rcl_interfaces::msg::SetParametersResult on_parameter_event(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    robot->lock();
    for (const auto &p : params) {
      const std::string &name = p.get_name();
      if (name == "TicksMM") {
        int v = p.as_int();
        if (v > 0 && v != TicksMM) {
          RCLCPP_INFO(get_logger(), "Setting TicksMM from parameter: %d -> %d", TicksMM, v);
          TicksMM = v;
          robot->comInt(93, TicksMM);
        }
      } else if (name == "DriftFactor") {
        int v = p.as_int();
        if (v != -99999 && v != DriftFactor) {
          RCLCPP_INFO(get_logger(), "Setting DriftFactor from parameter: %d -> %d", DriftFactor, v);
          DriftFactor = v;
          robot->comInt(89, DriftFactor);
        }
      } else if (name == "RevCount") {
        int v = p.as_int();
        if (v > 0 && v != RevCount) {
          RCLCPP_INFO(get_logger(), "Setting RevCount from parameter: %d -> %d", RevCount, v);
          RevCount = v;
          robot->comInt(88, RevCount);
        }
      } else if (name == "cmd_vel_timeout") {
        double v = p.as_double();
        if (v >= 0.0) {
          cmdvel_timeout = rclcpp::Duration::from_seconds(v);
        }
      } else if (name == "trans_accel" || name == "trans_decel" ||
                 name == "lat_accel" || name == "lat_decel" ||
                 name == "rot_accel" || name == "rot_decel")
      {
        // expecting these to be declared in the node externally if desired
        // We'll try to pick them up as doubles and apply the same scaling as ROS1:
        // trans/lat in m/s^2 -> Aria uses mm/s^2 (x1000); rotation given in radians -> convert to deg
        try {
          double val = p.as_double();
          int intv;
          if (name == "trans_accel") {
            intv = static_cast<int>(val * 1000.0);
            if (intv != robot->getTransAccel() && intv > 0) {
              RCLCPP_INFO(get_logger(), "Setting TransAccel: %d", intv);
              robot->setTransAccel(intv);
            }
          } else if (name == "trans_decel") {
            intv = static_cast<int>(val * 1000.0);
            if (intv != robot->getTransDecel() && intv > 0) {
              RCLCPP_INFO(get_logger(), "Setting TransDecel: %d", intv);
              robot->setTransDecel(intv);
            }
          } else if (name == "lat_accel") {
            intv = static_cast<int>(val * 1000.0);
            if (intv != robot->getLatAccel() && intv > 0) {
              if (robot->getAbsoluteMaxLatAccel() > 0)
                robot->setLatAccel(intv);
            }
          } else if (name == "lat_decel") {
            intv = static_cast<int>(val * 1000.0);
            if (intv != robot->getLatDecel() && intv > 0) {
              if (robot->getAbsoluteMaxLatDecel() > 0)
                robot->setLatDecel(intv);
            }
          } else if (name == "rot_accel") {
            intv = static_cast<int>(val * 180.0 / M_PI);
            if (intv != robot->getRotAccel() && intv > 0) {
              robot->setRotAccel(intv);
            }
          } else if (name == "rot_decel") {
            intv = static_cast<int>(val * 180.0 / M_PI);
            if (intv != robot->getRotDecel() && intv > 0) {
              robot->setRotDecel(intv);
            }
          }
        } catch (...) {
          // ignore miss-typed param
        }
      }
    }
    robot->unlock();
    return result;
  }

  void publish()
  {
    // Called by ARIA sensor interp thread (via ArRobot callback). Must be careful about ROS2 thread safety.
    // We'll build messages and then publish via rclcpp publishers (rclcpp is thread-safe for publish).
    if (!robot) return;

    // get pose (Aria returns mm and degrees)
    ArPose pos = robot->getPose();
    nav_msgs::msg::Odometry odom_msg;

    // pose -> transform
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = frame_id_odom;
    tf_msg.child_frame_id = frame_id_base_link;
    tf_msg.transform.translation.x = static_cast<double>(pos.getX()) / 1000.0;
    tf_msg.transform.translation.y = static_cast<double>(pos.getY()) / 1000.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, sin(pos.getTh()*M_PI/180.0/2.0), cos(pos.getTh()*M_PI/180.0/2.0)));

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = tf_msg.transform.translation.x;
    pose_msg.position.y = tf_msg.transform.translation.y;
    pose_msg.position.z = 0.0;
    pose_msg.orientation = tf_msg.transform.rotation;

    odom_msg.header.stamp = tf_msg.header.stamp;
    odom_msg.header.frame_id = frame_id_odom;
    odom_msg.child_frame_id = frame_id_base_link;
    odom_msg.pose.pose = pose_msg;

    // velocities
    odom_msg.twist.twist.linear.x = static_cast<double>(robot->getVel()) / 1000.0;
    odom_msg.twist.twist.linear.y = static_cast<double>(robot->getLatVel()) / 1000.0;
    odom_msg.twist.twist.angular.z = static_cast<double>(robot->getRotVel()) * M_PI / 180.0;

    // publish odom
    pose_pub->publish(odom_msg);

    // broadcast tf
    tf_broadcaster->sendTransform(tf_msg);

    // bumpers
    int stall = robot->getStallValue();
    unsigned char front_bumpers = static_cast<unsigned char>(stall >> 8);
    unsigned char rear_bumpers = static_cast<unsigned char>(stall);

    ros2aria_interfaces::msg::BumperState bumpers_msg;

    std::stringstream bumper_info(std::stringstream::out);
    // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
    for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
    {
      bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
      bumper_info << " " << (front_bumpers & (1 << (i+1)));
    }
    RCLCPP_WARN(get_logger(), "RosAria: Front bumpers:%s", bumper_info.str().c_str());

    bumper_info.str("");
    // Rear bumpers have reverse order (rightmost is LSB)
    unsigned int numRearBumpers = robot->getNumRearBumpers();
    for (unsigned int i=0; i<numRearBumpers; i++)
    {
      bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
      bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
    }
    RCLCPP_WARN(get_logger(), "RosAria: Rear bumpers:%s", bumper_info.str().c_str());



    bumpers_pub->publish(bumpers_msg);

    // battery
    std_msgs::msg::Float64 batteryVoltage;
    batteryVoltage.data = robot->getRealBatteryVoltageNow();
    voltage_pub->publish(batteryVoltage);

    if (robot->haveStateOfCharge()) {
      std_msgs::msg::Float32 soc;
      soc.data = robot->getStateOfCharge() / 100.0f;
      state_of_charge_pub->publish(soc);
    }

    char s = robot->getChargeState();
    std_msgs::msg::Int8 recharge_msg;
    recharge_msg.data = static_cast<int8_t>(s);
    // publish if changed
    if (recharge_msg.data != recharge_state.data) {
      recharge_state = recharge_msg;
      recharge_state_pub->publish(recharge_state);
    }

    bool motors_enabled = robot->areMotorsEnabled();
    if (!published_motors_state || motors_state.data != motors_enabled) {
      motors_state.data = motors_enabled;
      motors_state_pub->publish(motors_state);
      published_motors_state = true;
    }

    // Sonar: publish if subscriber count > 0
    bool pub_sonar = (sonar_pub->get_subscription_count() > 0);
    bool pub_sonar_pc2 = (sonar_pointcloud2_pub->get_subscription_count() > 0);

    if (pub_sonar || pub_sonar_pc2) {
      // enable sonar if needed
      robot->lock();
      robot->enableSonar();
      robot->unlock();

      sensor_msgs::msg::PointCloud cloud;
      cloud.header.stamp = odom_msg.header.stamp;
      cloud.header.frame_id = frame_id_sonar;

      for (int i = 0; i < robot->getNumSonar(); ++i) {
        ArSensorReading *reading = robot->getSonarReading(i);
        if (!reading) {
          RCLCPP_WARN(get_logger(), "RosAria: Did not receive a sonar reading.");
          continue;
        }
        geometry_msgs::msg::Point32 p;
        p.x = reading->getLocalX() / 1000.0f;
        p.y = reading->getLocalY() / 1000.0f;
        p.z = 0.0f;
        cloud.points.push_back(p);
      }

      if (pub_sonar_pc2) {
        sensor_msgs::msg::PointCloud2 cloud2;
        if (!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2)) {
          RCLCPP_WARN(get_logger(), "Error converting sonar point cloud message to PointCloud2 before publishing!");
        } else {
          sonar_pointcloud2_pub->publish(cloud2);
        }
      }

      if (pub_sonar) {
        sonar_pub->publish(cloud);
      }
    } else {
      // disable sonar if nobody's listening
      robot->lock();
      robot->disableSonar();
      robot->unlock();
    }
  }

  // cmd_vel callback
  void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    veltime = now();
    RCLCPP_INFO(get_logger(), "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x * 1e3, msg->angular.z, now().seconds());
    robot->lock();
    robot->setVel(static_cast<double>(msg->linear.x * 1e3));
    if (robot->hasLatVel()) robot->setLatVel(static_cast<double>(msg->linear.y * 1e3));
    robot->setRotVel(static_cast<double>(msg->angular.z * 180.0 / M_PI));
    robot->unlock();
  }

  // enable motors service
  void enable_motors_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(get_logger(), "RosAria: Enable motors request.");
    robot->lock();
    if (robot->isEStopPressed()) {
      RCLCPP_WARN(get_logger(), "Enable motors requested, but E-Stop is pressed. Motors will not enable.");
    }
    robot->enableMotors();
    robot->unlock();
  }

  // disable motors service
  void disable_motors_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(get_logger(), "RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
  }

  // watchdog: stop robot if cmd_vel timeout exceeded
  void cmdvel_watchdog()
  {
    if ((now() - veltime) > cmdvel_timeout) {
      robot->lock();
      robot->setVel(0.0);
      if (robot->hasLatVel()) robot->setLatVel(0.0);
      robot->setRotVel(0.0);
      robot->unlock();
    }
  }

  // helper to get rclcpp::Time now()
  rclcpp::Time now() {
    return this->get_clock()->now();
  }

}; // class RosAriaNode


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosAriaNode>());
  rclcpp::shutdown();
  return 0;
}
