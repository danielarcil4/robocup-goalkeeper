/**
 * @file main.c
 * @brief Application entry point and high level FreeRTOS tasks.
 *
 * Defines application-wide data structures, task entry points and the main
 * initialization sequence (app_main).
 */

#include <stdio.h>

#include "pid.h"
#include "init.h"
#include "motor.h"
#include "as5600.h"
#include "bno055.h"
#include "types_utils.h"

#include "wifi.h"
#include "ssl_receiver.h"
#include "ssl_parser.h"

void stop(motor_brushless_t * motor1, motor_brushless_t * motor2, motor_brushless_t * motor3);
void forward();
void back();
//void left();
//void right();

// Forward declaration for UART initialization if not included by a header
void uart_init_task(void);

// Forward declaration of the sensor reading task function
void vTaskReadSensors(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskUart(void* arg);
void vTaskUartHandler(void *arg);
void vTaskUartParser(void *arg);
void vTaskInverseKinematics(void *pvParameters);



motor_brushless_t motor[3]; ///< Array of brushless motors
AS5600_t as5600[3]; ///< Array of AS5600 sensors
BNO055_t bno055; ///< BNO055 sensor structure
adc_oneshot_unit_handle_t shared_adc_handle;


pid_block_handle_t pid[3]; ///< Array of PID controllers for each motor
pid_parameter_t pid_param = {
        .kp = PID_MOTOR_KP,
        .ki = PID_MOTOR_KI,
        .kd = PID_MOTOR_KD,
        .max_output = PID_MOTOR_MAX_OUTPUT, // Set maximum output for PID controller
        .min_output = PID_MOTOR_MIN_OUTPUT, // Set minimum output for PID controller
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .beta = PID_MOTOR_BETA // Set beta filter coefficient for derivative term
    };

RawSensorData sensor_data = {0};  //< Initialize with zeros
Velocity robot_command;           //< {vx, vy, wz}
Velocity robot_estimated;         //< {vx, vy, wz} - Estimated velocities from sensors
WheelSpeeds wheel_targets;  //< φ̇_1, φ̇_2, φ̇_3

SemaphoreHandle_t xCmdMutex;
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control
SemaphoreHandle_t xADCMutex = NULL; // Mutex for ADC operations
SemaphoreHandle_t xEstimatedDataMutex = NULL; // Mutex for estimated data
QueueHandle_t xPktQueue = NULL;
EventGroupHandle_t s_wifi_event_group = NULL;

TaskHandle_t xHandleParserTask;

#define CIRCULAR_RADIUS 1.0f
#define OMEGA_CIRC 0.5f                // Velocidad angular (rad/s)
#define DT_SECONDS 0.02f               // Periodo de actualización (segundos)
#define TASK_PERIOD_MS ((int)(DT_SECONDS * 1000))

/**
 * @brief Task that moves the motors forward and backward periodically.
 * 
 * This task alternates between setting the motors to move forward and backward
 * every second, using predefined setpoints for each motor.
 * 
 * @param arg Unused
 */
void vTaskMove(void* arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float t = 0.0f;
    float prev_angle = 0.0f;
    Velocity speed_estimated = {0};

    while (1) {
        // Calculate the command velocities based on a circular trajectory
        float vx_cmd = -CIRCULAR_RADIUS * OMEGA_CIRC * sinf(OMEGA_CIRC * t);
        float vy_cmd =  CIRCULAR_RADIUS * OMEGA_CIRC * cosf(OMEGA_CIRC * t);
        float angle = atan2f(vy_cmd, vx_cmd);
        float omega_cmd = (angle - prev_angle) / DT_SECONDS;

        if (omega_cmd > M_PI / DT_SECONDS) omega_cmd -= 2 * M_PI / DT_SECONDS;
        if (omega_cmd < -M_PI / DT_SECONDS) omega_cmd += 2 * M_PI / DT_SECONDS;

        prev_angle = angle;

        // Write the command to the shared robot_command structure
        if (xCmdMutex && xSemaphoreTake(xCmdMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            robot_command.vx = vx_cmd;
            robot_command.vy = vy_cmd;
            robot_command.wz = omega_cmd;
            xSemaphoreGive(xCmdMutex);
        }

        // Read the current robot state for debugging
        if (xEstimatedDataMutex && xSemaphoreTake(xEstimatedDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            speed_estimated = robot_estimated;
            xSemaphoreGive(xEstimatedDataMutex);
        }

        // Log the command for debugging every 1s
        if ((int)(t * 1000) % 1000 == 0) { // Log every second
            printf("Move Task, t=%.2f, vx=%.2f, vy=%.2f, wz=%.2f\n", t, vx_cmd, vy_cmd, omega_cmd);
            printf("Estimated Speed: vx=%.2f, vy=%.2f, wz=%.2f\n", 
                   speed_estimated.vx, speed_estimated.vy, speed_estimated.wz);
        }

        // Wait for the next cycle
        t += DT_SECONDS;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

void (*current_state)(void);

void vTaskTest(void * arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1){
        current_state();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}




void stop(motor_brushless_t * motor1,motor_brushless_t * motor2, motor_brushless_t * motor3){
    motor_stop(motor1);
    motor_stop(motor2);
    motor_stop(motor3);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void forward(){
    robot_command.vx = 0;
    robot_command.vy = 0.5;
    robot_command.wz = 0;
    vTaskDelay(pdMS_TO_TICKS(3000));
    //stop(&motor[0],&motor[1], NULL);
    current_state = back;
}




void back(){
    robot_command.vx = 0;
    robot_command.vy = -0.5;
    robot_command.wz = 0;
    vTaskDelay(pdMS_TO_TICKS(3000));
    //stop(&motor[0],&motor[1], NULL);
    current_state = forward;
}



/**
 * @brief Application entry point. Initializes peripherals, creates tasks and starts the scheduler.
 */
void app_main(void)
{

current_state = forward;
init_motors();  // Initialize motors
init_sensors(); // Initialize sensors
init_pid(); // Initialize PID controller
//uart_init_task(); // Initialize UART
// motor_calibration3(&motor[0], &motor[1], &motor[2]); // Calibrate all motors
vTaskDelay(pdMS_TO_TICKS(3000)); // Delay to allow sensors to stabilize

// Initialize the shared data mutex
xSensorDataMutex = xSemaphoreCreateMutex();
xPidMutex = xSemaphoreCreateMutex();
xADCMutex = xSemaphoreCreateMutex(); // Create mutex for ADC operations
xCmdMutex = xSemaphoreCreateMutex(); // Create mutex for command data
xEstimatedDataMutex = xSemaphoreCreateMutex(); // Create mutex for estimated data


// Start the sensor reading task with higher priority
xTaskCreate(vTaskReadSensors, "Sensor Task", 4096, NULL, 6, NULL);
// Start the control task with medium priority
xTaskCreate(vTaskControl, "Control Task", 4096, NULL, 3, NULL);
// // Start the inverse kinematics task with higher priority
xTaskCreate(vTaskInverseKinematics, "IK", 4096, NULL, 5, NULL);

// xTaskCreate(vTaskTest,"Test",4096, NULL, 7, NULL);

ESP_ERROR_CHECK(nvs_flash_init());

s_wifi_event_group = xEventGroupCreate();

// Crear cola de 10 paquetes
xPktQueue = xQueueCreate(10, sizeof(udp_packet_t));

// Tarea de WiFi
xTaskCreate(wifi_task, "wifi_task", 8192, NULL, 5, NULL);

// Esperar WiFi antes de arrancar socket
xEventGroupWaitBits(s_wifi_event_group,
                    WIFI_CONNECTED_BIT,
                    pdFALSE, pdFALSE,
                    portMAX_DELAY);

// Tareas del sistema SSL-Vision
xTaskCreate(ssl_receiver_task, "ssl_recv", 16386, NULL, 5, NULL);
xTaskCreate(ssl_parser_task,   "ssl_parse", 16386, NULL, 4, NULL);

// ESP_LOGI(TAG, "Sistema FreeRTOS inicializado.");

xTaskCreate(vTaskMove, "Move Task", 2048, NULL, 4, NULL); // Start the move task with lower priority
// // // Start the UART tunning the pid parameters task
// //xTaskCreate(vTaskUartHandler, "uart_handler", 4096, NULL, 10, NULL);
// //xTaskCreate(vTaskUartParser, "uart_parser", 4096, NULL, 10, &xHandleParserTask);
// robot_command.vx = 0.5;
// robot_command.vy = 0;
// robot_command.wz = 0;

// motor_set_speed(&motor[0], MOTOR_DIRECTION_FORWARD(0) * 30.0f); // Set motor 0 speed to 20% in forward direction
// motor_set_speed(&motor[1], MOTOR_DIRECTION_FORWARD(1) * 30.0f); // Set motor 1 speed to 20% in forward direction
// motor_set_speed(&motor[2], MOTOR_DIRECTION_FORWARD(2) * 30.0f); // Set motor 2 speed to 20% in forward direction
// forward();

}


