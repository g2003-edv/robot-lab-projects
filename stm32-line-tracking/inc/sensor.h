#ifndef __SENSOR_H
#define __SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define IR_SENSOR_NUM   5
#define OBSTACLE_THRESHOLD_CM  15.0f

typedef struct {
    uint8_t ir_values[IR_SENSOR_NUM];
    float   ultrasonic_distance_cm;
    int8_t  line_position;
} Sensor_Data;

void Sensor_Init(void);
void Sensor_ReadIR(Sensor_Data *data);
float Sensor_ReadUltrasonic(void);
uint8_t Sensor_IsObstacle(float distance);

#ifdef __cplusplus
}
#endif

#endif
