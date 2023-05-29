#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <driver/gpio.h>
#include "driver/ledc.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Macro functions
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10

// PINS
#define LED_BUILTIN 2
#define PIN_ESC 23
#define PIN_SERVO 22

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_SERVO LEDC_CHANNEL_1
#define PWM_ESC LEDC_CHANNEL_2

// Other PWM settings
#define PWM_FREQUENCY 50 
#define PWM_RESOLUTION LEDC_TIMER_12_BIT //Inverse relationship with f
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_R 410 // 2 ms width
#define PWM_MOTOR_N 307 // 1.5 ms width
#define PWM_MOTOR_F 205 // 1 ms width

#define PWM_SERVO_L 410 // 2 ms width //512 for 2.5 ms
#define PWM_SERVO_S 307 // 1.5 ms width
#define PWM_SERVO_R 205 //205 for 1 ms //0 for 0 ms

/*****************************************************************
The standard Frequency is used 50 HZ
With T=1/f the Period is 20 ms
Duty = 12 bit = 2^12 == 4096

The standard pulse of ESC and Servo are vary between 1 ms to 2 ms
D = pulse width / period

D = 2 / 20 == 10%
D = 1.5 /20 == 7.5 %
D = 1 /20 == 5%

PWM_2 	= 10% * 4096 	== 409.6
PWM_1.5	= 7.5% * 4096 	== 307.2
PWM_1	= 5% * 4096	== 204.8
*******************************************************************/

geometry_msgs__msg__Twist msg;


// Function forward declarations
void setupPins();
void setupRos();
void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);

// Main
void appMain(void *arg) 
{
    setupPins();
    setupRos();
}

void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);
    
    // Configure timer
    ledc_timer_config_t ledc_timer = 
    {
        .duty_resolution = PWM_RESOLUTION, //LEDC_TIMER_12_BIT
        .freq_hz         = PWM_FREQUENCY, //50 Hz. Berapa cycle (High & Off) per detik
        .speed_mode      = PWM_MODE, //High Speed Mode
        .timer_num       = PWM_TIMER, //LEDC_TIMER_1
        .clk_cfg         = LEDC_AUTO_CLK //Pake auto source clock, dependent sama given resolution and duty parameter pas init timer. Untuk limit pwm frequency. Higher source clock high maximum PWM yg bisa di configured
    };    
    ledc_timer_config(&ledc_timer);
    
    // Configure 2 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[2] = 
   {
        {
            .channel    = PWM_ESC, // LEDC_CHANNEL_2
            .duty       = 0, // set duty to 0%
            .gpio_num   = PIN_ESC, // GPIO 23
            .speed_mode = PWM_MODE, // LEDC_HIGH_SPEED_MODE
            .hpoint     = 0, // LEDC channel hpoint value, the max value is 0xfffff 
            .timer_sel  = PWM_TIMER // Select the timer source of channel (0 - 3) 
        },
        {
            .channel    = PWM_SERVO, //LEDC_CHANNEL_1
            .duty       = 0, // Rumusnya (Time per tick = (1/f) / 4096)
            .gpio_num   = PIN_SERVO, // GPIO 22
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = PWM_TIMER
        },
    };

    for (int i = 0; i < 2; i++) 
    {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void setupRos() {
    // Initialize micro-ROS allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support; // Initialize support object

    printf("Sampe line 144\n");
    
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "F1Tenth_Drive", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(FRAME_TIME),
        timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME * 1000);
    }

    // free resources
    
    //After finishing the publisher/subscriber, the node will no longer be advertising that it is publishing/listening on the topic.
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    
    //To destroy an initialized timer. This will deallocate used memory and make the timer invalid
    RCCHECK(rcl_node_fini(&node)); 

    vTaskDelete(NULL);
}

// We don't really need the callback, because msg is set anyway
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;
    printf("Message received: %f %f\n", msg->linear.x, msg->angular.z);
}

// Each frame, check msg data and set PWM channels accordingly
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) 
{

    if (timer == NULL) {
        return;
    }

    printf("LED is ON\n");
    gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    
// Then map those values to PWM intensities.

    // Use linear.x for forward value and angular.z for rotation
    float linear = constrain(msg.linear.x, -1, 1);
    float angular = constrain(msg.angular.z, -1, 1);

    // This robot is an RC tank and uses a differential drive (skid steering).
    // Calculate the speed of left and right motors. Simple version without wheel distances.
    // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
     
     // Driving
     uint16_t pwmForward = (uint16_t) fmap(fabs(linear), 0, 1, PWM_MOTOR_N, PWM_MOTOR_F); // Forward
     uint16_t pwmReverse = (uint16_t) fmap(fabs(linear), 0, 1, PWM_MOTOR_N, PWM_MOTOR_R); // Reverse
     uint16_t pwmNeutral = (uint16_t) fmap(fabs(linear), 0, 1, PWM_MOTOR_N, PWM_MOTOR_N); // Neutral
     
     // Steering
     uint16_t pwmLeft = (uint16_t) fmap(fabs(angular), 0, 1, PWM_SERVO_S, PWM_SERVO_L); // Left
     uint16_t pwmRight = (uint16_t) fmap(fabs(angular), 0, 1, PWM_SERVO_S, PWM_SERVO_R); // Right
     uint16_t pwmStraight = (uint16_t) fmap(fabs(angular), 0, 1, PWM_SERVO_S, PWM_SERVO_S); // Straight
     
     /* 
     
     Note: 
     Full period is 20 ms; 
     
     Servo width
     1 ms = 0 degree; 1.5 ms = 90 degree ; 2 ms = 180 degree
     
     ESC width
     1 ms = Forward ; 1.5 ms = Neutral ; 2 ms = Reverse
     
     */
     
     if (linear > 0) /* Forward Drive */
     {
     	ledc_set_duty(PWM_MODE, PWM_ESC, pwmForward);
     	ledc_update_duty(PWM_MODE, PWM_ESC);
     	printf ("pwmForward value is %d\n", pwmForward);
     	
     	if (angular == 0) /* Straight Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmStraight);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmStraight value is %d\n", pwmStraight);
     	}
     	else if (angular > 0) /* Left Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmLeft);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmLeft value is %d\n", pwmLeft);
     	}
     	else if (angular < 0) /* Right Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmRight);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmRight value is %d\n", pwmRight);
     	}

     }
     else if (linear < 0) /* Reverse Drive */
     {
     	ledc_set_duty(PWM_MODE, PWM_ESC, pwmReverse);
     	ledc_update_duty(PWM_MODE, PWM_ESC);
     	printf ("pwmReverse value is %d\n", pwmReverse);
     	
     	if (angular == 0) /* Straight Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmStraight);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmStraight value is %d\n", pwmStraight);
     	}
     	else if (angular > 0) /* Left Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmLeft);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmLeft value is %d\n", pwmLeft);
     	}
     	else if (angular < 0) /* Right Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmRight);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmRight value is %d\n", pwmRight);
     	}

     	
     }
     else if (linear == 0) /* Neutral Drive */
     {
    	ledc_set_duty(PWM_MODE, PWM_ESC, pwmNeutral);
    	ledc_update_duty(PWM_MODE, PWM_ESC);
    	printf ("pwmNeutral value is %d\n", pwmNeutral);
     	
     	if (angular == 0) /* Straight Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmStraight);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmStraight value is %d\n", pwmStraight);
     	}
     	else if (angular > 0) /* Left Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmLeft);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmLeft value is %d\n", pwmLeft);
     	}
     	else if (angular < 0) /* Right Steer */
     	{
     		ledc_set_duty(PWM_MODE, PWM_SERVO, pwmRight);
     		ledc_update_duty(PWM_MODE, PWM_SERVO);
     		printf ("pwmRight value is %d\n", pwmRight);
     	}
     }
}


// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max) 
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
