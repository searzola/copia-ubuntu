// ----- Run microros node -----
// ls /dev/serial/by-id/*                                                  -> get serial device name
// sudo chmod a+wr [serial device name]                                    -> give permissions to serial device
// ros2 run micro_ros_agent micro_ros_agent serial --dev [nombre del com]  -> start micro_ros agent and microros controller node

// ---- Send topic messages on terminal -----
// ros2 topic pub --once /wamv_caleuche/thrusters/turn_on_off std_msgs/msg/Int16 data:\ 1\    -> turn on the motor
// ros2 topic pub --once /wamv_caleuche/thrusters/turn_on_off std_msgs/msg/Int16 data:\ 0\    -> turn off the motor
// ros2 topic pub --once /wamv_caleuche/thrusters/test2/cmd_vel std_msgs/msg/Int16 data:\ X\  -> set the motor speed to X (-1000 < X < 1000)


// String messages management based on:
// https://github.com/uhobeike/micro-ros-st-nucleo-f446re/blob/master/Core/Src/main.c#L92

#include "variables.hpp"

const int input_vcc = 21; // Relay pin //! Por Implementar
int16_t cmd_vel = 0; // Motor velocity command

// microROS handles
unsigned int num_handles = 2;   // 2 subscribers

void setup()
{
    // ---- ESP SETUP ----
    pinMode(LED_BUILTIN, OUTPUT);    // Error LED
    digitalWrite(LED_BUILTIN, LOW);  // If there is an error, the LED will turn on
    delay(1000);
    
    int baud_rate = 115200;
    Serial.begin(baud_rate);
    
    set_microros_serial_transports(Serial);
    delay(1000);

    state = WAITING_AGENT;
}

void loop()
{
    microros_loop();
    //motor.loop(cmd_vel);
}


// ------------------------
// ------- MICROROS -------
// ------------------------

bool microros_create_entities(){
    
    // Node information
    const char *node_name = "thruster_test";
    const char *node_ns = ""; //namespace
    
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
    RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node

    // ---- MICROROS PUBLISHER ----
    RCCHECK(rclc_publisher_init_default( // create publisher
      &feedback_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "/wamv_caleuche/thrusters/test/feedback"));

    // ---- MICROROS SUBSCRIBERS ----
    RCCHECK(rclc_subscription_init_default( // topic for turning on and off the thruster
        &turn_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), 
        "/wamv_caleuche/thrusters/turn_on_off"));
    
    RCCHECK(rclc_subscription_init_default( // topic for giving velocity commands to the thruster
        &cmd_vel_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/wamv_caleuche/thrusters/test/cmd_vel"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &turn_sub, &turn_msg, &sub_turn_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &sub_cmd_vel_callback, ON_NEW_DATA));

    return true;
}

// Destroys all microROS entities
void microros_destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    rcl_publisher_fini(&feedback_publisher, &node);
    rcl_subscription_fini(&turn_sub, &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Verifies and restablishes connection with microROS agent
void microros_loop(){
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 6)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == microros_create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        microros_destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 6)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        if (Serial2.available() > 0) {
          RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
          RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_pub_msg, NULL));
        } 
      }
      break;
      
    case AGENT_DISCONNECTED:
      microros_destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}

// Runs every time a message is received on /wamv_caleuche/thrusters/turn_on_off
// Turns the motor on or off
void sub_turn_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    turn_msg.data = msg->data;
    
    if (turn_msg.data) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    } 
}

// Runs every time a message is received on /wamv_caleuche/thrusters/test2/cmd_vel
// Updates the motor velocity command
void sub_cmd_vel_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    cmd_vel_msg.data = msg->data;
    feedback_pub_msg.data = cmd_vel_msg.data;
    cmd_vel = cmd_vel_msg.data;
}

// ROS Error Handle Loop: If there is an error, the LED will turn on
void error_loop() {
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}
