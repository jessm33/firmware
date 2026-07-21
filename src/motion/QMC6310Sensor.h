#pragma once
#ifndef _QMC6310_SENSOR_H_
#define _QMC6310_SENSOR_H_

#include "MotionSensor.h"

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && __has_include(<SensorQMC6310.hpp>)

#include <SensorQMC6310.hpp>

class QMC6310Sensor : public MotionSensor
{
  private:
    SensorQMC6310 mag;
    bool showingScreen = false;
    static constexpr const char *compassCalibrationFileName = "/prefs/compass_qmc6310.dat";
    float highestX = 0, lowestX = 0, highestY = 0, lowestY = 0, highestZ = 0, lowestZ = 0;

  public:
    explicit QMC6310Sensor(ScanI2C::FoundDevice foundDevice);

    virtual bool init() override;
    virtual int32_t runOnce() override;
    virtual void calibrate(uint16_t forSeconds) override;
    virtual bool providesHeading() const override { return true; }
};

#endif

#endif
