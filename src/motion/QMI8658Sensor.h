#pragma once
#ifndef _QMI8658_SENSOR_H_
#define _QMI8658_SENSOR_H_

#include "MotionSensor.h"

#if !defined(ARCH_STM32WL) && defined(HAS_QMI8658) && __has_include(<SensorQMI8658.hpp>)

#include <SensorQMI8658.hpp>

class QMI8658Sensor : public MotionSensor
{
  private:
    SensorQMI8658 imu;

  public:
    explicit QMI8658Sensor(ScanI2C::FoundDevice foundDevice);

    // Initialise the IMU over SPI; ignores device.address (SPI, not I2C)
    virtual bool init() override;

    // Reads accelerometer and publishes a sample for tilt-compensated compass heading
    virtual int32_t runOnce() override;
};

#endif

#endif
