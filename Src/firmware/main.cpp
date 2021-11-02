#include <cstring>

#include <ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

#include "usart.h"

#include "firmware/configuration.hpp"
#include "firmware/parameters.hpp"
#include "firmware/wheel_controller.hpp"

ros::NodeHandle nh;

static std_msgs::Float32 battery;
static ros::Publisher *battery_pub;
static bool publish_battery = false;

static geometry_msgs::TwistStamped odom;
static ros::Publisher *odom_pub;
static geometry_msgs::PoseStamped pose;
static ros::Publisher *pose_pub;
static bool publish_odom = false;

static sensor_msgs::JointState joint_states;
static ros::Publisher *joint_states_pub;
static bool publish_joint = false;

static DiffDriveController dc(DD_CONFIG);

void cmdVelCallback(const geometry_msgs::Twist &msg) {
  dc.setSpeed(msg.linear.x, msg.angular.z);
}

void resetOdometryCallback(const std_srvs::TriggerRequest &req,
                           std_srvs::TriggerResponse &res) {
  dc.resetOdom();
  res.success = true;
}

void getFirmwareCallback(const std_srvs::TriggerRequest &req,
                         std_srvs::TriggerResponse &res) {
  res.message = FIRMWARE_VERSION;
  res.success = true;
}

void initROS() {
  // Publishers
  battery_pub = new ros::Publisher("battery", &battery);
  odom_pub = new ros::Publisher("wheel_odom", &odom);
  pose_pub = new ros::Publisher("wheel_pose", &pose);
  joint_states_pub = new ros::Publisher("joint_states", &joint_states);

  nh.advertise(*battery_pub);
  nh.advertise(*odom_pub);
  nh.advertise(*pose_pub);
  nh.advertise(*joint_states_pub);

  // Subscribers
  auto twist_sub =
      new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &cmdVelCallback);

  nh.subscribe(*twist_sub);

  // Services
  auto reset_odometry_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                   std_srvs::TriggerResponse>(
      "core2/reset_odometry", &resetOdometryCallback);
  auto firmware_version_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                     std_srvs::TriggerResponse>(
      "core2/get_firmware_version", &getFirmwareCallback);

  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      *reset_odometry_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      *firmware_version_srv);
}

void setupJoints() {
  static char *joint_names[] = {
      (char *)"wheel_FL_joint", (char *)"wheel_RL_joint",
      (char *)"wheel_FR_joint", (char *)"wheel_RR_joint"};
  joint_states.name_length = 4;
  joint_states.name = joint_names;
  joint_states.position_length = 4;
  joint_states.position = dc.positions;
  joint_states.velocity_length = 4;
  joint_states.velocity = dc.velocities;
  joint_states.effort_length = 4;
  joint_states.effort = dc.efforts;
}

void setupOdom() {
  odom.header.frame_id = params.robot_frame_id;
  pose.header.frame_id = params.odom_frame_id;
}

void fmain() {
  nh.getHardware()->setUart(&ROSSERIAL_UART);
  nh.initNode();

  initROS();

  // Wait for rosserial connection
  while (!nh.connected()) {
    nh.spinOnce();
  }

  params.load(nh);

  setupJoints();
  setupOdom();

  // Initialize Diff Drive Controller
  dc.init();

  while (true) {
    nh.spinOnce();

    if (!nh.connected()) continue;

    if (publish_battery) {
      battery_pub->publish(&battery);
      publish_battery = false;
    }

    if (publish_odom) {
      odom_pub->publish(&odom);
      pose_pub->publish(&pose);
      publish_odom = false;
    }

    if (publish_joint) {
      joint_states_pub->publish(&joint_states);
      publish_joint = false;
    }
  }
}