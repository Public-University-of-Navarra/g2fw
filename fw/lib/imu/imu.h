#ifndef IMU_H
#define IMU_H

struct imu_data_t
{
    float ax, ay, az; // acceleration, m/s^2
    float gx, gy, gz; // rotation rate, deg/s
};

int imu_begin();
int imu_update();
int imu_get(imu_data_t *out); // copies the last valid sample; FW_ERR_NOT_READY if none yet

#endif