//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "main.h"
#include "spi.h"
#include "Math/Math.h"
#include "BMI323_Platform.h"

typedef enum
{
    SENSOR1_I,
    SENSOR2_I,
    SENSOR3_I
} BMI_INDEX;

struct BMI_Data
{
    Vector3D<float> accel_linear;
    Vector3D<float> ang_vel;

    // Raw register values (LSB), no unit conversion
    Vector3D<int16_t> accel_raw;
    Vector3D<int16_t> gyro_raw;
    int16_t temp_raw = 0;

    // incremented every time we pull data from each channel: accel/gyro/temp
    // each channel may update at different times, so sometimes BMI_Data might be
    // fresh for one sensor but stale for others. use these counts to help determine
    // what's fresh and what's not. mostly important for logging
    // uint64's so that we don't overflow
    uint64_t accel_update_count = 0;
    uint64_t gyro_update_count = 0;
    uint64_t temp_update_count = 0;
};

typedef struct
{
    SPI_HandleTypeDef *SPIbus;
    BMI_INDEX index;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} BMIobjPtr;

class SIMU_BMI323
{
public:
    SIMU_BMI323();
    int Init(BMI_INDEX DeviceNum);
    int Update();
    BMI_Data getRawData();

private:
    bmi3_dev device;
    bool _sensor_active;
    BMI_INDEX InitDev;
    BMI_Data _raw;
    BMIobjPtr _obj;

    bmi3_sens_config config[2];
    bmi3_map_int map_int = {0};

    /*! @brief Converts raw sensor values(LSB) to G value
     *
     *  @param[in] val        : Raw sensor value.
     *  @param[in] g_range    : Accel Range selected (4G).
     *  @param[in] bit_width  : Resolution of the sensor.
     *
     *  @return Accel values in Gravity(G)
     *
     */
    static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width);

    /*!
     * @brief This function converts lsb to degree per second for 16 bit gyro at
     * range 125, 250, 500, 1000 or 2000dps.
     */
    static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
};

#endif // KINGFISHER_SW_SIMU_BMI323_H