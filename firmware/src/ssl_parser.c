/**
 * @file ssl_parser.c
 * @brief Parsing helpers to extract geometry and detection messages from protobuf wrappers.
 */
#include "ssl_parser.h"

/**
 * @brief Log a single robot's information from a detection frame.
 *
 * Helper used by `process_detection` to print robot fields such as
 * confidence, optional ID and pose.
 *
 * @param robot Pointer to the protobuf robot message
 */
void print_robot_info(const SSL_DetectionRobot *robot) {
    ESP_LOGI(TAG_WIFI, "    CONF=%.2f", robot->confidence);
    if (robot->has_robot_id) {
        ESP_LOGI(TAG_WIFI, "    ID=%u", (unsigned)robot->robot_id);
    } else {
        ESP_LOGI(TAG_WIFI, "    ID=N/A");
    }
    ESP_LOGI(TAG_WIFI, "    POS=(%.2f, %.2f)", robot->x, robot->y);
    if (robot->has_orientation) {
        ESP_LOGI(TAG_WIFI, "    ANGLE=%.3f rad", robot->orientation);
    }
}

/**
 * @brief Process a detection frame and log its contents.
 *
 * Iterates over detected balls and robots and logs their properties.
 * Primarily used for debugging and validation of incoming vision packets.
 *
 * @param detection Pointer to the decoded detection frame
 */
void process_detection(const SSL_DetectionFrame *detection) {
    ESP_LOGI(TAG_WIFI, "=== Detection Frame ===");
    ESP_LOGI(TAG_WIFI, "Camera: %u, Frame: %u", 
             (unsigned)detection->camera_id, 
             (unsigned)detection->frame_number);
    ESP_LOGI(TAG_WIFI, "T_CAPTURE: %.4f, T_SENT: %.4f",
             detection->t_capture, detection->t_sent);
    
    /* Note: In nanopb repeated arrays use pb_size_t for counts
       field name is: <field_name>_count */
    
    /* Process balls */
    ESP_LOGI(TAG_WIFI, "Balls detected: %u", (unsigned)detection->balls_count);
    for (pb_size_t i = 0; i < detection->balls_count; i++) {
        const SSL_DetectionBall *ball = &detection->balls[i];
        ESP_LOGI(TAG_WIFI, "  Ball %u: CONF=%.2f POS=(%.2f, %.2f)", 
                 (unsigned)i, ball->confidence, ball->x, ball->y);
        if (ball->has_z) {
            ESP_LOGI(TAG_WIFI, "    Z=%.2f", ball->z);
        }
    }
    
    /* Process blue robots */
    ESP_LOGI(TAG_WIFI, "Blue robots: %u", (unsigned)detection->robots_blue_count);
    for (pb_size_t i = 0; i < detection->robots_blue_count; i++) {
        ESP_LOGI(TAG_WIFI, "  Blue Robot %u:", (unsigned)i);
        print_robot_info(&detection->robots_blue[i]);
    }
    
    /* Process yellow robots */
    ESP_LOGI(TAG_WIFI, "Yellow robots: %u", (unsigned)detection->robots_yellow_count);
    for (pb_size_t i = 0; i < detection->robots_yellow_count; i++) {
        ESP_LOGI(TAG_WIFI, "  Yellow Robot %u:", (unsigned)i);
        print_robot_info(&detection->robots_yellow[i]);
    }
} 

/**
 * @brief Process and log field geometry information from a geometry message.
 *
 * Logs dimensions of the field and optional parameters such as penalty area size
 * and center circle radius.
 *
 * @param geometry Pointer to the decoded geometry message
 */
void process_geometry(const SSL_GeometryData *geometry) {
    ESP_LOGI(TAG_WIFI, "=== Geometry Data ===");
    
    const SSL_GeometryFieldSize *field = &geometry->field;
    ESP_LOGI(TAG_WIFI, "Field dimensions:");
    ESP_LOGI(TAG_WIFI, "  Length: %d mm, Width: %d mm", 
             field->field_length, field->field_width);
    ESP_LOGI(TAG_WIFI, "  Goal Width: %d mm, Depth: %d mm", 
             field->goal_width, field->goal_depth);
    
    if (field->has_penalty_area_depth) {
        ESP_LOGI(TAG_WIFI, "  Penalty area: %d x %d mm", 
                 field->penalty_area_depth, field->penalty_area_width);
    }
    
    if (field->has_center_circle_radius) {
        ESP_LOGI(TAG_WIFI, "  Center circle radius: %d mm", field->center_circle_radius);
    }
} 