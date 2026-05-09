#include "odometry.h"
#include <math.h>

#define PI 3.14159265f

void Odometry_Init(Odometry_t *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
     odom->last_enc1 = 0;
    odom->last_enc3 = 0;
}

void Odometry_Update(Odometry_t *odom, int32_t enc1_val, int32_t enc3_val, float angle_deg) {
    // 1. Calculate Pulse Change (Δn)
    int32_t dn1 = enc1_val - odom->last_enc1;
    int32_t dn3 = enc3_val - odom->last_enc3;
    odom->last_enc1 = enc1_val;
    odom->last_enc3 = enc3_val;
    if (dn1 > 32768)  dn1 -= 65536;
        if (dn1 < -32768) dn1 += 65536;

        if (dn3 > 32768)  dn3 -= 65536;
        if (dn3 < -32768) dn3 += 65536;
    // 2. Encoder Conversion (S = 2πR * Δn/N) [cite: 25]
    float S1 = (2.0f * PI * ODOM_R) * ((float)dn1 / ODOM_N);
    float S3 = (2.0f * PI * ODOM_R) * ((float)dn3 / ODOM_N);

    // 3. Heading Change (Δθ) [cite: 13]
    float current_theta_rad = angle_deg * (PI / 180.00f);
    float delta_theta = current_theta_rad - odom->theta;

    // 4. Local Displacement [cite: 17, 18]
    // Δx = S1 + (L) * Δθ
    // Δy = S3 - B * Δθ
    float dx_local = S1 + (ODOM_L * delta_theta);
    float dy_local = S3 - (ODOM_B * delta_theta);

    // 5. Global Update (Midpoint Integration) [cite: 20, 21, 22]
    float theta_m = odom->theta + (delta_theta / 2.0f);

    odom->x += dx_local * cosf(theta_m) - dy_local * sinf(theta_m);
    odom->y += dx_local * sinf(theta_m) + dy_local * cosf(theta_m);
    odom->theta = current_theta_rad;
}
