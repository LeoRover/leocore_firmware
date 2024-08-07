#include <cstdio>
#include <cstring>
#include <string>

#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <leo_msgs/Imu.h>
#include <leo_msgs/WheelOdom.h>
#include <leo_msgs/WheelOdomMecanum.h>
#include <leo_msgs/WheelStates.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include "mainf.h"

#include "diff_drive_lib/diff_drive_controller.hpp"
#include "diff_drive_lib/mecanum_controller.hpp"
#include "diff_drive_lib/wheel_controller.hpp"

#include "firmware/configuration.hpp"
#include "firmware/imu_receiver.hpp"
#include "firmware/parameters.hpp"

static ros::NodeHandle nh;
static bool configured = false;

static std_msgs::Float32 battery;
static std_msgs::Float32 battery_averaged;
static ros::Publisher battery_pub("firmware/battery", &battery);
static ros::Publisher battery_averaged_pub("firmware/battery_averaged",
                                           &battery);
static diff_drive_lib::CircularBuffer<float> battery_buffer_(
    BATTERY_BUFFER_SIZE);
static bool publish_battery = false;

static leo_msgs::WheelOdom wheel_odom;
static ros::Publisher wheel_odom_pub("firmware/wheel_odom", &wheel_odom);
static leo_msgs::WheelOdomMecanum wheel_odom_mecanum;
static ros::Publisher wheel_odom_mecanum_pub("firmware/wheel_odom_mecanum",
                                             &wheel_odom_mecanum);
static bool publish_wheel_odom = false;

static leo_msgs::WheelStates wheel_states;
static ros::Publisher wheel_states_pub("firmware/wheel_states", &wheel_states);
static bool publish_wheel_states = false;

static leo_msgs::Imu imu;
static ros::Publisher imu_pub("firmware/imu", &imu);
static bool publish_imu = false;

static bool reset_request = false;

MotorController MotA(MOT_A_CONFIG);
MotorController MotB(MOT_B_CONFIG);
MotorController MotC(MOT_C_CONFIG);
MotorController MotD(MOT_D_CONFIG);

static diff_drive_lib::RobotController *controller;
static ImuReceiver imu_receiver(&IMU_I2C);

Parameters params;

void cmdVelCallback(const geometry_msgs::Twist &msg) {
  controller->setSpeed(msg.linear.x, msg.linear.y, msg.angular.z);
}

void resetOdometryCallback(const std_srvs::TriggerRequest &req,
                           std_srvs::TriggerResponse &res) {
  controller->resetOdom();
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
  res.message = "leocore";
  res.success = true;
}

struct WheelWrapper {
  explicit WheelWrapper(diff_drive_lib::WheelController &wheel,
                        std::string wheel_name)
      : wheel_(wheel),
        cmd_pwm_topic("firmware/wheel_" + wheel_name + "/cmd_pwm_duty"),
        cmd_vel_topic("firmware/wheel_" + wheel_name + "/cmd_velocity"),
        cmd_pwm_sub_(cmd_pwm_topic.c_str(), &WheelWrapper::cmdPWMDutyCallback,
                     this),
        cmd_vel_sub_(cmd_vel_topic.c_str(), &WheelWrapper::cmdVelCallback,
                     this) {}

  void initROS() {
    nh.subscribe(cmd_pwm_sub_);
    nh.subscribe(cmd_vel_sub_);
  }

  void cmdPWMDutyCallback(const std_msgs::Float32 &msg) {
    wheel_.disable();
    wheel_.motor.setPWMDutyCycle(msg.data);
  }

  void cmdVelCallback(const std_msgs::Float32 &msg) {
    wheel_.enable();
    wheel_.setTargetVelocity(msg.data);
  }

 private:
  diff_drive_lib::WheelController &wheel_;
  std::string cmd_pwm_topic;
  std::string cmd_vel_topic;
  ros::Subscriber<std_msgs::Float32, WheelWrapper> cmd_pwm_sub_;
  ros::Subscriber<std_msgs::Float32, WheelWrapper> cmd_vel_sub_;
};

static WheelWrapper *wheel_FL_wrapper;
static WheelWrapper *wheel_RL_wrapper;
static WheelWrapper *wheel_FR_wrapper;
static WheelWrapper *wheel_RR_wrapper;

static ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel",
                                                       &cmdVelCallback);

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

void initROS() {
  // Publishers
  nh.advertise(battery_pub);
  nh.advertise(battery_averaged_pub);
  if (params.mecanum_wheels) {
    nh.advertise(wheel_odom_mecanum_pub);
  } else {
    nh.advertise(wheel_odom_pub);
  }
  nh.advertise(wheel_states_pub);
  nh.advertise(imu_pub);

  // Subscribers
  nh.subscribe(twist_sub);

  // Services
  nh.advertiseService(reset_odometry_srv);
  nh.advertiseService(firmware_version_srv);
  nh.advertiseService(board_type_srv);
  nh.advertiseService(reset_board_srv);

  wheel_FL_wrapper->initROS();
  wheel_RL_wrapper->initROS();
  wheel_FR_wrapper->initROS();
  wheel_RR_wrapper->initROS();
}

void setup() {
  nh.getHardware()->setUart(&ROSSERIAL_UART);
  nh.initNode();

  // Wait for rosserial connection
  while (!nh.connected()) {
    nh.spinOnce();
  }

  params.load(nh);
  if (params.mecanum_wheels) {
    controller = new diff_drive_lib::MecanumController(ROBOT_CONFIG);
  } else {
    controller = new diff_drive_lib::DiffDriveController(ROBOT_CONFIG);
  }

  wheel_FL_wrapper = new WheelWrapper(controller->wheel_FL, "FL");
  wheel_RL_wrapper = new WheelWrapper(controller->wheel_RL, "RL");
  wheel_FR_wrapper = new WheelWrapper(controller->wheel_FR, "FR");
  wheel_RR_wrapper = new WheelWrapper(controller->wheel_RR, "RR");

  initROS();

  imu_receiver.init();

  // Initialize Robot Controller
  controller->init(params);

  configured = true;
}

void loop() {
  nh.spinOnce();

  if (!nh.connected()) return;

  if (publish_battery) {
    battery_pub.publish(&battery);
    battery_averaged_pub.publish(&battery_averaged);
    publish_battery = false;
  }

  if (publish_wheel_odom) {
    if (params.mecanum_wheels) {
      wheel_odom_mecanum_pub.publish(&wheel_odom_mecanum);
    } else {
      wheel_odom_pub.publish(&wheel_odom);
    }
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
  ++cnt;

  static float battery_sum = 0.0F;
  static float battery_avg = 0.0F;
  float battery_new = static_cast<float>(BATTERY_ADC) * BATTERY_ADC_TO_VOLTAGE;
  battery_sum += battery_new;
  battery_sum -= battery_buffer_.push_back(battery_new);
  battery_avg =
      battery_sum / static_cast<float>(std::min(BATTERY_BUFFER_SIZE, cnt));

  if (battery_avg < params.battery_min_voltage) {
    if (cnt % 10 == 0) gpio_toggle(LED);
  } else {
    if (!nh.connected()) {
      if (cnt % 50 == 0) gpio_toggle(LED);
    } else {
      gpio_reset(LED);
    }
  }

  if (!configured) return;

  controller->update(UPDATE_PERIOD);

  if (!nh.connected()) return;

  if (reset_request) {
    delay(1000);
    reset();
  }

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = battery_new;
    battery_averaged.data = battery_avg;

    publish_battery = true;
  }

  if (cnt % JOINTS_PUB_PERIOD == 0 && !publish_wheel_states) {
    auto dd_wheel_states = controller->getWheelStates();

    wheel_states.stamp = nh.now();
    for (size_t i = 0; i < 4; i++) {
      wheel_states.position[i] = dd_wheel_states.position[i];
      wheel_states.velocity[i] = dd_wheel_states.velocity[i];
      wheel_states.torque[i] = dd_wheel_states.torque[i];
      wheel_states.pwm_duty_cycle[i] = dd_wheel_states.pwm_duty_cycle[i];
    }

    publish_wheel_states = true;
  }

  if (cnt % ODOM_PUB_PERIOD == 0 && !publish_wheel_odom) {
    auto dd_odom = controller->getOdom();

    if (params.mecanum_wheels) {
      wheel_odom_mecanum.stamp = nh.now();
      wheel_odom_mecanum.velocity_lin_x = dd_odom.velocity_lin_x;
      wheel_odom_mecanum.velocity_lin_y = dd_odom.velocity_lin_y;
      wheel_odom_mecanum.velocity_ang = dd_odom.velocity_ang;
      wheel_odom_mecanum.pose_x = dd_odom.pose_x;
      wheel_odom_mecanum.pose_y = dd_odom.pose_y;
      wheel_odom_mecanum.pose_yaw = dd_odom.pose_yaw;
    } else {
      wheel_odom.stamp = nh.now();
      wheel_odom.velocity_lin = dd_odom.velocity_lin_x;
      wheel_odom.velocity_ang = dd_odom.velocity_ang;
      wheel_odom.pose_x = dd_odom.pose_x;
      wheel_odom.pose_y = dd_odom.pose_y;
      wheel_odom.pose_yaw = dd_odom.pose_yaw;
    }
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