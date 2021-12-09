#include <cstdio>
#include <cstring>

#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <leo_msgs/Imu.h>
#include <leo_msgs/WheelOdom.h>
#include <leo_msgs/WheelStates.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include "mainf.h"

#include "firmware/configuration.hpp"
#include "firmware/imu_receiver.hpp"
#include "firmware/parameters.hpp"
#include "firmware/wheel_controller.hpp"

static ros::NodeHandle nh;
static bool configured = false;

static std_msgs::Float32 battery;
static ros::Publisher battery_pub("firmware/battery", &battery);
static bool publish_battery = false;

static leo_msgs::WheelOdom wheel_odom;
static ros::Publisher wheel_odom_pub("firmware/wheel_odom", &wheel_odom);
static bool publish_wheel_odom = false;

static leo_msgs::WheelStates wheel_states;
static ros::Publisher wheel_states_pub("firmware/wheel_states", &wheel_states);
static bool publish_wheel_states = false;

static leo_msgs::Imu imu;
static ros::Publisher imu_pub("firmware/imu", &imu);
static bool publish_imu = false;

static bool reset_request = false;

static DiffDriveController dc(DD_CONFIG);
static ImuReceiver imu_receiver(&IMU_I2C);

void cmdVelCallback(const geometry_msgs::Twist &msg) {
  dc.setSpeed(msg.linear.x, msg.angular.z);
}

void resetOdometryCallback(const std_srvs::TriggerRequest &req,
                           std_srvs::TriggerResponse &res) {
  dc.resetOdom();
  res.success = true;
}

void resetBoardCallback(const std_srvs::TriggerRequest &req,
                        std_srvs::TriggerResponse &res) {
  reset_request = true;
  res.message = "Requested board software reset";
  res.success = true;
}

void getFirmwareVersionCallback(const std_srvs::TriggerRequest &req,
                                std_srvs::TriggerResponse &res) {
  res.message = FIRMWARE_VERSION;
  res.success = true;
}

void getBoardTypeCallback(const std_srvs::TriggerRequest &req,
                          std_srvs::TriggerResponse &res) {
  res.message = "leo_hat";
  res.success = true;
}

void initROS() {
  // Publishers
  nh.advertise(battery_pub);
  nh.advertise(wheel_odom_pub);
  nh.advertise(wheel_states_pub);
  nh.advertise(imu_pub);

  // Subscribers
  static ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel",
                                                         &cmdVelCallback);

  nh.subscribe(twist_sub);

  // Services
  using TriggerService =
      ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>;

  static TriggerService reset_odometry_srv("firmware/reset_odometry",
                                           &resetOdometryCallback);
  static TriggerService firmware_version_srv("firmware/get_firmware_version",
                                             &getFirmwareVersionCallback);
  static TriggerService board_type_srv("firmware/get_board_type",
                                       &getBoardTypeCallback);
  static TriggerService reset_board_srv("firmware/reset_board",
                                        &resetBoardCallback);

  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      reset_odometry_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      firmware_version_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      board_type_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      reset_board_srv);
}

void setup() {
  nh.getHardware()->setUart(&ROSSERIAL_UART);
  nh.initNode();

  initROS();

  // Wait for rosserial connection
  while (!nh.connected()) {
    nh.spinOnce();
  }

  imu_receiver.init();

  params.load(nh);

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

  if (publish_wheel_odom) {
    wheel_odom_pub.publish(&wheel_odom);
    publish_wheel_odom = false;
  }

  if (publish_wheel_states) {
    wheel_states_pub.publish(&wheel_states);
    publish_wheel_states = false;
  }

  if (publish_imu) {
    imu_pub.publish(&imu);
    publish_imu = false;
  }
}

void update() {
  static uint32_t cnt = 0;

  if (reset_request) {
    delay(1000);
    reset();
  }

  if (!configured || !nh.connected()) return;

  ++cnt;
  dc.update(UPDATE_PERIOD);

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = static_cast<float>(BATTERY_ADC) * BATTERY_ADC_TO_VOLTAGE;

    publish_battery = true;
  }

  if (cnt % JOINTS_PUB_PERIOD == 0 && !publish_wheel_states) {
    wheel_states.stamp = nh.now();
    dc.updateWheelStates(wheel_states);

    publish_wheel_states = true;
  }

  if (cnt % ODOM_PUB_PERIOD == 0 && !publish_wheel_odom) {
    wheel_odom = dc.getOdom();
    wheel_odom.stamp = nh.now();

    publish_wheel_odom = true;
  }

  if (cnt % IMU_PUB_PERIOD == 0 && !publish_imu) {
    imu_receiver.update();

    imu.stamp = nh.now();
    imu.temperature = imu_receiver.temp;
    imu.accel_x = imu_receiver.ax;
    imu.accel_y = imu_receiver.ay;
    imu.accel_z = imu_receiver.az;
    imu.gyro_x = imu_receiver.gx;
    imu.gyro_y = imu_receiver.gy;
    imu.gyro_z = imu_receiver.gz;

    publish_imu = true;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &ROSSERIAL_UART) {
    nh.getHardware()->TxCpltCallback();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &ROSSERIAL_UART) {
    nh.initNode();
  }
}