#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>

#include <test_msgs/srv/basic_types.h>

// ------------------
// Setup wifi
// ------------------
#include "credentials.h"

// ----------------------
// Error handling macros
// ----------------------
#define RCCHECK(fn, error_handler)    \
  {                                   \
    rcl_ret_t temp_rc = fn;           \
    if ((temp_rc != RCL_RET_OK)) {    \
      Serial.printf("(!=%d) [line %d] [%s]", temp_rc, __LINE__, rcutils_get_error_string().str); \
      error_handler;                  \
    }                                 \
  }

#define RCCRETRY(fn)                  \
  {                                   \
    rcl_ret_t temp_rc;                \
    do {                              \
      temp_rc = fn;                   \
    } while (temp_rc != RCL_RET_OK);  \
  }

#define RCSOFTCHECK(fn)               \
  {                                   \
    rcl_ret_t temp_rc = fn;           \
    if ((temp_rc != RCL_RET_OK)) {    \
      Serial.printf("(!=%d) [line %d] [%s]", temp_rc, __LINE__, rcutils_get_error_string().str); \
    }                                 \
  }


// --------------------
// PIN initialization
// --------------------
#define LED_PIN 2


// --------------------
// Node initialization
// --------------------
bool uros_initialized = false;
rcl_node_t node;
rclc_support_t support;
rcl_init_options_t init_options;
rcl_allocator_t allocator;
rclc_executor_t executor;
int num_execution_handles = 2; // Update this based on how many handlers will be registered (right now it is 1 service handler + 1 timer callback)

// ------------------
// Service objects
// ------------------
rcl_service_t service;
test_msgs__srv__BasicTypes_Request request;
test_msgs__srv__BasicTypes_Response response;

// ------------------
// Publisher objects
// ------------------
rcl_timer_t timer;
rcl_publisher_t publisher;
std_msgs__msg__String msg;
int counter = 0;

// ------------------
// Teardown
// ------------------
void teardown_ros() {
  if (uros_initialized) {
    rcl_ret_t rc = rcl_publisher_fini(&publisher, &node);
    rc += rcl_service_fini(&service, &node);
    rc += rcl_node_fini(&node);
    rc += rcl_timer_fini(&timer);
    rc += rclc_executor_fini(&executor);
    rc += rclc_support_fini(&support);
    micro_ros_utilities_destroy_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      &msg,
      micro_ros_utilities_memory_conf_default
    );
    micro_ros_utilities_destroy_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, srv, BasicTypes_Request),
      &request,
      micro_ros_utilities_memory_conf_default
    );
    micro_ros_utilities_destroy_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, srv, BasicTypes_Response),
      &response,
      micro_ros_utilities_memory_conf_default
    );
    if (rc != RCL_RET_OK) {
      Serial.println("Error while cleaning up!");
    }
    uros_initialized = false;
  }
}

// ------------------
// Error loop
// ------------------
void error_loop() {
  teardown_ros();
  while (1) {
    Serial.println("error loop");
    delay(200);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// ------------------
// Timed state publisher
// ------------------
void timed_publish_trigger(rcl_timer_t* timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    sprintf(msg.data.data, "%d", counter++);
    msg.data.size = strlen(msg.data.data);
    Serial.printf("^ [%s]", msg.data.data);
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  } else {
    Serial.print("_");
  }
}

// ------------------
// Control service handler
// ------------------
void service_callback(const void * req_in, void * res_in) {
  Serial.print("~");
  test_msgs__srv__BasicTypes_Request * _request = (test_msgs__srv__BasicTypes_Request *) req_in;
  test_msgs__srv__BasicTypes_Response * _response = (test_msgs__srv__BasicTypes_Response *) res_in;
  _response->int16_value = _request->int16_value - 1;
//  sprintf(_response->string_value.data, "%s", "blablabla");
//  _response->string_value.size = strlen(_response->string_value.data);
  Serial.printf("Control/State: %d / %d~\n", (int) _request->int16_value, (int) _response->int16_value);
}

// ------------------
// Initialization
// ------------------
void initialize_ros() {
  Serial.print("Initializing ros");
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, (uint)agent_port);
  
  // create allocator
  Serial.print(".");
  allocator = rcl_get_default_allocator();

  // create init_options
  Serial.print(".");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator), error_loop());

  // setup node
  Serial.print(".");
  node = rcl_get_zero_initialized_node();
  Serial.print(".");
  RCCHECK(rclc_node_init_default(&node, "control_node", "robot", &support), error_loop());

  // setup executor
  Serial.print(".");
  executor = rclc_executor_get_zero_initialized_executor();
  Serial.print(".");
  RCCHECK(rclc_executor_init(&executor, &support.context, num_execution_handles, &allocator), error_loop());
  Serial.print(".");
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(2000)), error_loop());

  // setup service
  Serial.print(".");
  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, srv, BasicTypes_Request), &request, micro_ros_utilities_memory_conf_default);
  Serial.print(".");
  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, srv, BasicTypes_Response), &response, micro_ros_utilities_memory_conf_default);
  Serial.print(".");
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(test_msgs, srv, BasicTypes), "/control"), error_loop());
  Serial.print(".");
  RCCHECK(rclc_executor_add_service(&executor, &service, (void*) &request, (void*) &response, &service_callback), error_loop());

  // setup publisher
  const unsigned int timer_timeout_ms = 5000; // ms
  Serial.print(".");
  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), &msg, micro_ros_utilities_memory_conf_default);
  Serial.print(".");
  RCCRETRY(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "chatter"));
  Serial.print(".");
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout_ms), &timed_publish_trigger), error_loop());
  Serial.print(".");
  RCCHECK(rclc_executor_add_timer(&executor, &timer), error_loop());

  Serial.print(".");
  // RCCHECK(rmw_uros_sync_session(5000), error_loop());

  uros_initialized = true;

  Serial.println("Done");
}

// ------------------
// Responsiveness variables
// ------------------
int loop_frequency = 10;                        // Hz


// ------------------
// Setup()
// ------------------
void setup() {
  // Setup pins
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);

  Serial.println("Initializing:");
  delay(3000);

  initialize_ros();

  digitalWrite(LED_PIN, HIGH);
  Serial.println("Startup complete!");
  Serial.print("[Looping]");
}

// ------------------
// Loop()
// ------------------
void loop() {
  int delay_ms = (1.0 / loop_frequency) * 1000;
  delay(delay_ms);

  Serial.print("\n. ");
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // 1 second timeout to check for msgs on DDS queue
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
