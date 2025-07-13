#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"




#include <stdio.h>

#include "pid.h"
#include "init.h"
#include "motor.h"
#include "as5600.h"
#include "bno055.h"
#include "udp_server.h"
#include "types_utils.h"

static const char* TAG_UDP2 = 'TAG_UDP';

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
WheelSpeeds wheel_targets;  //< φ̇_1, φ̇_2, φ̇_3

SemaphoreHandle_t xCmdMutex;
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control
SemaphoreHandle_t xADCMutex = NULL; // Mutex for ADC operations

TaskHandle_t xHandleParserTask;

#define CIRCULAR_RADIUS 0.5f
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

 instr_t instr_stop = {
    .cmd = 'S',
};
QueueHandle_t xInstrQueue2;


void vMove(instr_t instruction, float *vx_i, float *vy_i, float *prev_angle_i, float *omega_i, float t_i, uint32_t *delay){
    // Calculate the command velocities based on a circular trajectory
    float angle;
    float rad = M_PI*instruction.angle/180;
    switch (instruction.cmd)
    {
    case 'L':

            *vx_i = (instruction.direction)*instruction.velocity * cosf(rad);
            *vy_i = (instruction.direction)*instruction.velocity * sinf(rad);
            *omega_i = 0;
            *delay = (uint32_t)(instruction.distance*10/instruction.velocity);
        break;
    case 'R':
            *vx_i = 0.0;
            *vy_i = 0.0;
            *omega_i = (instruction.direction)*instruction.velocity;
            *delay = (uint32_t)(instruction.angle*M_PI*100/instruction.velocity/18);
        break;
    case 'C':
        *vx_i = -instruction.angle * instruction.velocity * sinf(instruction.velocity  * t_i);
        *vy_i =  instruction.angle * instruction.velocity * cosf(instruction.velocity  * t_i);
        angle = atan2f(*vy_i, *vx_i);
        *omega_i = (angle - *prev_angle_i) / DT_SECONDS;

        if (*omega_i > M_PI / DT_SECONDS) *omega_i -= 2 * M_PI / DT_SECONDS;
        if (*omega_i < -M_PI / DT_SECONDS) *omega_i += 2 * M_PI / DT_SECONDS;
        *prev_angle_i = angle;
        *delay = M_PI*1e6/instruction.velocity;
        break;
    case 'S': //STOP
        *vx_i = 0.0;
        *vy_i = 0.0;
        *omega_i = 0.0;
    break;
    default:
        *vx_i = 0.0;
        *vy_i = 0.0;
        *omega_i = 0.0;
        break;
    }
    
}
void vTaskMove(void* arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    instr_t instr_2_process;
    float t = 0.0f;
    float prev_angle = 0.0f;

    float vx_cmd, vy_cmd, omega_cmd;
    uint32_t d;
    

    while (1) {
        if(xInstrQueue2 != NULL){
            if(xQueueReceive(xInstrQueue2,&instr_2_process,pdMS_TO_TICKS(500)) == pdPASS){
                ESP_LOGI(TAG_UDP2, "Instruction execute");
                ESP_LOGI(TAG_UDP2, "\nCMD : %c\nDirection: %d\nDist: %f\nVel: %f\nAngle : %f\n", instr_2_process.cmd,instr_2_process.direction,instr_2_process.distance,instr_2_process.velocity,instr_2_process.angle);
                // Write the command to the shared robot_command structure
                if(!instr_2_process.direction) instr_2_process.direction = -1;
                int64_t tiempo_in_us = esp_timer_get_time();
                    while(1){
                        vMove(instr_2_process,&vx_cmd,&vy_cmd,&prev_angle,&omega_cmd, t, &d);
                        if (xCmdMutex && xSemaphoreTake(xCmdMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                            robot_command.vx = vx_cmd;
                            robot_command.vy = vy_cmd;
                            robot_command.wz = omega_cmd;
                            xSemaphoreGive(xCmdMutex);
                        }
                        if(instr_2_process.cmd== 'C'){
                            t += DT_SECONDS;
                            if((esp_timer_get_time()-tiempo_in_us) >= d){
                                break;
                            }
                            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
                        }
                        else{
                            
                            ESP_LOGI(TAG_UDP2, "Delay %d", (uint16_t)d);
                            vTaskDelay(pdMS_TO_TICKS(d));
                            break;
                        }
                    }
                    if (xCmdMutex && xSemaphoreTake(xCmdMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                        robot_command.vx = 0;
                        robot_command.vy = 0;
                        robot_command.wz = 0;
                        xSemaphoreGive(xCmdMutex);
                    }
            }
            else{
                 ESP_LOGI(TAG_UDP2, "Instruction queue empty");
            }
        }
        else{
            ESP_LOGE(TAG_UDP2, "Queue doesnt exist");
            vTaskDelay(pdMS_TO_TICKS(1e4));
        }
        

        
    }
}

void app_main(void)
{
    init_motors();  // Initialize motors
    init_sensors(); // Initialize sensors
    init_pid(); // Initialize PID controller
    init_comm(); // initialize Communications
    xInstrQueue2 = return_queue_handle();
    // uart_init_task(); // Initialize UART
    // motor_calibration3(&motor_0, &motor_1, &motor_2); // Calibrate all motors
    vTaskDelay(pdMS_TO_TICKS(3000)); // Delay to allow sensors to stabilize

    // Initialize the shared data mutex
    xSensorDataMutex = xSemaphoreCreateMutex();
    xPidMutex = xSemaphoreCreateMutex();
    xADCMutex = xSemaphoreCreateMutex(); // Create mutex for ADC operations
    xCmdMutex = xSemaphoreCreateMutex(); // Create mutex for command data


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