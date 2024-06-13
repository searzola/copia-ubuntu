// add arduino libs
#include <Arduino.h>
#include <Tasker.h>
#include <ServoInput.h>
#include <stdio.h>

// add microros libs
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/trigger.h>

// add extra libs 
#include "crc8/crc8.h"
#include "torqeedo/torqeedo.h"


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t turn_sub;
std_msgs__msg__Bool turn_msg;

rcl_subscription_t cmd_vel_sub;
std_msgs__msg__Int16 cmd_vel_msg;

rcl_service_t service;
std_srvs__srv__Trigger_Request request_msg;
std_srvs__srv__Trigger_Response response_msg;

rcl_timer_t timer;
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} //{error_loop();}}

void sub_turn_callback(const void * msgin);
void sub_cmd_vel_callback(const void * msgin);
void service_callback(const void * req_msg, void * res_msg);
bool microros_create_entities();
