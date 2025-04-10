#include "wallaby_imu.h"

#include <MPU9250.h>

#include "wallaby.h"

float AccData[3], GyroData[3], MagData[3];

void pack_value(const uint8_t reg, const float value)
{
    const uint32_t intValue = *(uint32_t*)&value; // Reinterpret the float as an integer
    aTxBuffer[reg]  = (intValue & 0xFF00) >> 8;
    aTxBuffer[reg + 1] = intValue & 0x00FF;
}

void setupIMU()
{
    if (MPU9250_Init() != 0) return;

    MPU9250_SetAccelRange(ACCEL_RANGE_2G);
    MPU9250_SetGyroRange(GYRO_RANGE_2000DPS);
    MPU9250_SetDLPFBandwidth(DLPF_BANDWIDTH_5HZ);
}

void readIMU()
{
    MPU9250_GetData(AccData, MagData, GyroData);

    pack_value(REG_RW_GYRO_X_H, GyroData[0]);
    pack_value(REG_RW_GYRO_Y_H, GyroData[1]);
    pack_value(REG_RW_GYRO_Z_H, GyroData[2]);

    pack_value(REG_RW_ACCEL_X_H, AccData[0]);
    pack_value(REG_RW_ACCEL_Y_H, AccData[1]);
    pack_value(REG_RW_ACCEL_Z_H, AccData[2]);

    pack_value(REG_RW_MAG_X_H, MagData[0]);
    pack_value(REG_RW_MAG_Y_H, MagData[1]);
    pack_value(REG_RW_MAG_Z_H, MagData[2]);
}
