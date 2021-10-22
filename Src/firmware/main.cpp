#include <cstring>
#include <string>

#include <ros.h>

#include <std_msgs/Bool.h>

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

ros::NodeHandle nh;

static std_msgs::Bool bool1;
static ros::Publisher *test_pub;

void initROS() {
  test_pub = new ros::Publisher("test", &bool1);
  nh.advertise(*test_pub);
}

void fmain() {
  nh.getHardware()->setUart(&huart2);
  nh.initNode();

  // Wait for rosserial connection
  while (!nh.connected()) {
    nh.spinOnce();
  }

  initROS();

  while (true) {
    bool1.data = !bool1.data;
    test_pub->publish(&bool1);
    nh.spinOnce();
    HAL_Delay(10);
  }
}