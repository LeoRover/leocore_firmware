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

static ros::NodeHandle nh;
static bool configured = false;

static std_msgs::Float32 battery;
static ros::Publisher battery_pub("battery", &battery);
static bool publish_battery = false;

static geometry_msgs::TwistStamped odom;
static ros::Publisher odom_pub("wheel_odom", &odom);
static geometry_msgs::PoseStamped pose;
static ros::Publisher pose_pub("wheel_pose", &pose);
static bool publish_odom = false;

static sensor_msgs::JointState joint_states;
static ros::Publisher joint_states_pub("joint_states", &joint_states);
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
  nh.advertise(battery_pub);
  nh.advertise(odom_pub);
  nh.advertise(pose_pub);
  nh.advertise(joint_states_pub);

  // Subscribers
  static ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel",
                                                         &cmdVelCallback);

  nh.subscribe(twist_sub);

  // Services
  static ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
      reset_odometry_srv("core2/reset_odometry", &resetOdometryCallback);
  static ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
      firmware_version_srv("core2/get_firmware_version", &getFirmwareCallback);

  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      reset_odometry_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      firmware_version_srv);
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

void setup() {
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

  configured = true;
}

void loop() {
  nh.spinOnce();

  if (!nh.connected()) return;

  if (publish_battery) {
    battery_pub.publish(&battery);
    publish_battery = false;
  }

  if (publish_odom) {
    odom_pub.publish(&odom);
    pose_pub.publish(&pose);
    publish_odom = false;
  }

  if (publish_joint) {
    joint_states_pub.publish(&joint_states);
    publish_joint = false;
  }
}

void update() {
  static uint32_t cnt = 0;

  if (!configured || !nh.connected()) return;

  ++cnt;
  dc.update(10);

  if (cnt % 5 == 0 && !publish_joint) {
    joint_states.header.stamp = nh.now();
    dc.updateWheelStates();

    publish_joint = true;
  }

  if (cnt % 5 == 0 && !publish_odom) {
    odom.header.stamp = nh.now();

    Odom odo = dc.getOdom();
    odom.twist.linear.x = odo.vel_lin;
    odom.twist.angular.z = odo.vel_ang;
    pose.pose.position.x = odo.pose_x;
    pose.pose.position.y = odo.pose_y;
    pose.pose.orientation.z = std::sin(odo.pose_yaw * 0.5F);
    pose.pose.orientation.w = std::cos(odo.pose_yaw * 0.5F);

    publish_odom = true;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  nh.getHardware()->reset_rbuf();
}