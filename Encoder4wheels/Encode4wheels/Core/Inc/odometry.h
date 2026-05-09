#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"

// Constants from Odometry_Summary.docx [cite: 6, 7, 11]
#define ODOM_R    0.030f    // Radius
#define ODOM_N    8600.0f   // CPR
#define ODOM_L    0.0f     // Wheelbase
#define ODOM_B    0.20f     // Lateral Offset

typedef struct {
    float x;
    float y;
    float theta;
    int32_t last_enc1;
    int32_t last_enc3;
    float dx_local;
    float dy_local;
} Odometry_t;

void Odometry_Init(Odometry_t *odom);
void Odometry_Update(Odometry_t *odom, int32_t enc_4val, int32_t enc3_val, float angle_deg);

#endif
