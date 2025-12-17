file(GLOB SRC_FILES "../src/*.c"
     GLOB TASK_FILES "../tasks/*.c"
     GLOB UTILS_FILES "../utils/*.c"
     GLOB PROTOBUF_FILES "../protobuf/*.c")  # Include all .c files in /src

idf_component_register(
    SRCS "init.c" "main.c" ${SRC_FILES} ${TASK_FILES} ${UTILS_FILES} ${PROTOBUF_FILES}   # Add main.c and all files from src
    INCLUDE_DIRS "." "../include" "../utils" "../protobuf"
    REQUIRES lwip nanopb esp_wifi esp_netif esp_event nvs_flash driver esp_timer esp_adc
)
