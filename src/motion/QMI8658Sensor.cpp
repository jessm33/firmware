#include "QMI8658Sensor.h"

#if !defined(ARCH_STM32WL) && defined(HAS_QMI8658) && __has_include(<SensorQMI8658.hpp>)

// SPI_HSPI is created by setupSDCard() in FSCommon.cpp when SDCARD_USE_SPI1 is defined.
// The QMI8658 shares the same HSPI bus as the SD card slot.
#if defined(HAS_SDCARD) && defined(SDCARD_USE_SPI1)
#include "FSCommon.h"
extern SPIClass SPI_HSPI;
#endif

static constexpr int32_t QMI8658_UPDATE_INTERVAL_MS = 50;

QMI8658Sensor::QMI8658Sensor(ScanI2C::FoundDevice foundDevice) : MotionSensor::MotionSensor(foundDevice) {}

bool QMI8658Sensor::init()
{
#if defined(HAS_SDCARD) && defined(SDCARD_USE_SPI1)
    // Hold SD card CS high before any HSPI transaction so a missing or
    // unpowered card cannot float the bus and corrupt the IMU transfer.
    pinMode(SPI_CS, OUTPUT);
    digitalWrite(SPI_CS, HIGH);
    LOG_DEBUG("QMI8658 begin on SPI HSPI cs=%d", QMI8658_SPI_CS);
    if (!imu.begin(SPI_HSPI, QMI8658_SPI_CS)) {
        LOG_DEBUG("QMI8658 init error");
        return false;
    }
#else
    LOG_DEBUG("QMI8658 requires SDCARD_USE_SPI1 (HSPI bus) - not available on this build");
    return false;
#endif

    imu.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_250Hz, SensorQMI8658::LPF_MODE_0);
    imu.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_224_2Hz, SensorQMI8658::LPF_MODE_0);

    return true;
}

int32_t QMI8658Sensor::runOnce()
{
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (imu.getAccelerometer(ax, ay, az)) {
        publishCompassAccelSample(ax, ay, az);
    }
    return QMI8658_UPDATE_INTERVAL_MS;
}

#endif
