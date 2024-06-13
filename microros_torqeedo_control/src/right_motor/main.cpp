// ----- Run microros node -----
// ls /dev/serial/by-id/*                                                  -> get serial device name
// sudo chmod a+wr [serial device name]                                    -> give permissions to serial device
// ros2 run micro_ros_agent micro_ros_agent serial --dev [nombre del com]  -> start micro_ros agent and microros controller node

// ---- Send topic messages on terminal ---sudo chmod a+wr --
// ros2 topic pub --once /wamv_caleuche/thrusters/right/turn_on_off std_msgs/msg/Bool data:\ true\    -> turn off the motor
// ros2 topic pub --once /wamv_caleuche/thrusters/right/cmd_vel std_msgs/msg/Int16 data:\ X\  -> set the motor speed to X (-1000 < X < 1000)


// String messages management based on:
// https://github.com/uhobeike/micro-ros-st-nucleo-f446re/blob/master/Core/Src/main.c#L92

#include "variables.hpp"

CRC8 crc8;            // Initialize CRC8-Maxim calculation class
TorqeedoMotor motor;  // Initialize TorqeedoMotor class

const int input_vcc = 21; // Relay pin //! Por Implementar
//bool motor_status = false; // False = Motor apagado | True = Motor encendido //! Por Implementar
int16_t cmd_vel = 0; // Motor velocity command


// microROS handles
unsigned int num_handles = 2;   // 2 subscribers

void setup()
{
    // ---- ESP SETUP ----
    pinMode(LED_BUILTIN, OUTPUT);    // Error LED
    digitalWrite(LED_BUILTIN, LOW);  // If there is an error, the LED will turn on
    delay(1000);

     // ---- MOTOR PCB SETUP ---- 
    pinMode(input_vcc, INPUT);
    motor.begin(2, 19, 18, 33);  // 1=ESPSerial, Tx=TX2, Rx=RX2, OnOff=33)
    delay(1000); 

    int baud_rate = 115200;
    Serial2.begin(baud_rate);
    set_microros_serial_transports(Serial2);
    delay(5000);
    
    // wait until agent is aviailable
    while ((RMW_RET_ERROR == rmw_uros_ping_agent(100, 1))) {
        delay(1000);
    }
  
    microros_create_entities();
}

void loop()
{
  if (Serial2.available() > 0) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  } 

  motor.loop(cmd_vel);
}


// ------------------------
// ------- MICROROS -------
// ------------------------

bool microros_create_entities(){
    
    // Node information
    const char *node_name = "thruster_right";
    const char *node_ns = ""; //namespace
    
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
    RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node

    // ---- MICROROS SUBSCRIBERS ----
    RCCHECK(rclc_subscription_init_default( // topic for turning on and off the thruster
        &turn_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), 
        "/wamv_caleuche/thrusters/right/turn_on_off"));
    
    RCCHECK(rclc_subscription_init_default( // topic for giving velocity commands to the thruster
        &cmd_vel_sub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/wamv_caleuche/thrusters/right/cmd_vel"));

    // ---- MICROROS SERVICES ----
    const char * service_name = "/right_motor_feedback";
    const rosidl_service_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
    RCCHECK(rclc_service_init_default(&service, &node, type_support, service_name));

    // ---- MICROROS EXECUTOR ----
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &turn_sub, &turn_msg, &sub_turn_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &sub_cmd_vel_callback, ON_NEW_DATA));

    return true;
}

// Runs every time a message is received on /wamv_caleuche/thrusters/turn_on_off
// Turns the motor on or off
void sub_turn_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    turn_msg.data = msg->data;
    
    if (turn_msg.data) {
        motor.On();
    }
    else {
        motor.Off();
    } 
}

// Runs every time a message is received on /wamv_caleuche/thrusters/right/cmd_vel
// Updates the motor velocity command
void sub_cmd_vel_callback(const void * msgin){
    // Cast message pointer to expected type
    std_msgs__msg__Int16 * msg = (std_msgs__msg__Int16 *)msgin;
    cmd_vel_msg.data = msg->data;
    cmd_vel = cmd_vel_msg.data;
}

void service_callback(const void * req_msg, void * res_msg){
    // Cast messages to expected types
    std_srvs__srv__Trigger_Request * req_in =(std_srvs__srv__Trigger_Request *) req_msg;
    std_srvs__srv__Trigger_Response * res_out = (std_srvs__srv__Trigger_Response *) res_msg;
    
    // microros string message response

    float rpm_f = (float)motor.feedback_getMotorRPM();
    String fabc(rpm_f, 0);
    const char* rpm_str = fabc.c_str();
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(rpm_str);  
    response_msg.message.data = ros_str.data;

    // Handle request message and set the response message values
    res_out->success = true;
    res_out->message.data = response_msg.message.data;
}