#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <stdio.h>

#include "init.h"
#include "motor.h"
#include "as5600.h"
#include "bno055.h"
#include "udp_server.h"
#include "types_utils.h"

// Forward declaration for UART initialization if not included by a header
void uart_init_task(void);

// Forward declaration of the sensor reading task function
void vTaskReadSensors(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskUart(void* arg);
void vTaskUartHandler(void *arg);
void vTaskUartParser(void *arg);
void vTaskInverseKinematics(void *pvParameters);
void vTaskMove(void *arg);

motor_brushless_t motor[3]; ///< Array of brushless motors
AS5600_t as5600[3]; ///< Array of AS5600 sensors
BNO055_t bno055; ///< BNO055 sensor structure
adc_oneshot_unit_handle_t shared_adc_handle;

SemaphoreHandle_t xCmdMutex;
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control
SemaphoreHandle_t xADCMutex = NULL; // Mutex for ADC operations
SemaphoreHandle_t xI2CMutex = NULL;

TaskHandle_t xHandleParserTask;

void app_main(void)
{
    init_motors();  // Initialize motors
    init_sensors(); // Initialize sensors
    init_pid(); // Initialize PID controller
    init_comm(); // initialize Communications
    // uart_init_task(); // Initialize UART
    // motor_calibration3(&motor_0, &motor_1, &motor_2); // Calibrate all motors
    vTaskDelay(pdMS_TO_TICKS(3000)); // Delay to allow sensors to stabilize

    // Initialize the shared data mutex
    xSensorDataMutex = xSemaphoreCreateMutex();
    xPidMutex = xSemaphoreCreateMutex();
    xADCMutex = xSemaphoreCreateMutex(); // Create mutex for ADC operations
    xCmdMutex = xSemaphoreCreateMutex(); // Create mutex for command data
    xI2CMutex = xSemaphoreCreateMutex();

    // Start the sensor reading task with higher priority
    xTaskCreate(vTaskReadSensors, "Sensor Task", 4096, NULL, 6, NULL);
    // Start the control task with medium priority
    xTaskCreate(vTaskControl, "Control Task", 4096, NULL, 3, NULL);
    // Start the inverse kinematics task with higher priority
    xTaskCreate(vTaskInverseKinematics, "IK", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskMove, "Move Task", 4096, NULL, 4, NULL); // Start the move task with lower priority
    // // Start the UART tunning the pid parameters task
    // xTaskCreate(vTaskUartHandler, "uart_handler", 4096, NULL, 10, NULL);
    // xTaskCreate(vTaskUartParser, "uart_parser", 4096, NULL, 10, &xHandleParserTask);
    
}