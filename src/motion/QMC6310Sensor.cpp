#include "QMC6310Sensor.h"

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && __has_include(<SensorQMC6310.hpp>)

#include "Fusion/Fusion.h"
#include "detect/ScanI2CTwoWire.h"

#if !defined(MESHTASTIC_EXCLUDE_SCREEN)
extern graphics::Screen *screen;
#endif

static constexpr float QMC6310_HEADING_OFFSET_DEG = 180.0f;
static constexpr int32_t QMC6310_UPDATE_INTERVAL_MS = 20;
static constexpr uint32_t QMC6310_ACCEL_STALE_MS = 300;
static constexpr float QMC6310_MIN_AXIS_RADIUS = 1e-4f;

QMC6310Sensor::QMC6310Sensor(ScanI2C::FoundDevice foundDevice) : MotionSensor::MotionSensor(foundDevice) {}

bool QMC6310Sensor::init()
{
    LOG_DEBUG("QMC6310 begin on addr 0x%02x (port=%d)", device.address.address, device.address.port);
    TwoWire *wire = ScanI2CTwoWire::fetchI2CBus(device.address);

    // Pass sda=-1, scl=-1 so SensorLib reuses the already-initialised Wire bus.
    if (!mag.begin(*wire, device.address.address)) {
        LOG_DEBUG("QMC6310 init error");
        return false;
    }

    mag.configMagnetometer(SensorQMC6310::MODE_CONTINUOUS, SensorQMC6310::RANGE_8G, SensorQMC6310::DATARATE_200HZ,
                           SensorQMC6310::OSR_1, SensorQMC6310::DSR_1);

    loadMagnetometerCalibration(compassCalibrationFileName, highestX, lowestX, highestY, lowestY, highestZ, lowestZ);
    return true;
}

int32_t QMC6310Sensor::runOnce()
{
    if (!mag.isDataReady())
        return QMC6310_UPDATE_INTERVAL_MS;

    if (mag.readData() < 0) {
        LOG_DEBUG("QMC6310 read error");
        return QMC6310_UPDATE_INTERVAL_MS;
    }

    float magX = mag.getX();
    float magY = mag.getY();
    float magZ = mag.getZ();

#if !defined(MESHTASTIC_EXCLUDE_SCREEN)
    if (doCalibration) {
        beginCalibrationDisplay(showingScreen);
        updateCalibrationExtrema(magX, magY, magZ, highestX, lowestX, highestY, lowestY, highestZ, lowestZ);
        finishCalibrationIfExpired(showingScreen, compassCalibrationFileName, highestX, lowestX, highestY, lowestY, highestZ,
                                   lowestZ);
    }
#endif

    // Hard-iron bias removal
    magX -= (highestX + lowestX) * 0.5f;
    magY -= (highestY + lowestY) * 0.5f;
    magZ -= (highestZ + lowestZ) * 0.5f;

    // Soft-iron diagonal scaling from calibration extrema
    const float radiusX = (highestX - lowestX) * 0.5f;
    const float radiusY = (highestY - lowestY) * 0.5f;
    const float radiusZ = (highestZ - lowestZ) * 0.5f;
    const float avgRadius = (radiusX + radiusY + radiusZ) / 3.0f;
    magX *= (radiusX > QMC6310_MIN_AXIS_RADIUS) ? (avgRadius / radiusX) : 1.0f;
    magY *= (radiusY > QMC6310_MIN_AXIS_RADIUS) ? (avgRadius / radiusY) : 1.0f;
    magZ *= (radiusZ > QMC6310_MIN_AXIS_RADIUS) ? (avgRadius / radiusZ) : 1.0f;

#if !defined(MESHTASTIC_EXCLUDE_SCREEN) && HAS_SCREEN
    float heading;
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
    uint32_t accelAgeMs = 0;

    if (getLatestCompassAccelSample(accelX, accelY, accelZ, accelAgeMs) && accelAgeMs <= QMC6310_ACCEL_STALE_MS) {
        FusionVector ga = {.axis = {accelX, accelY, accelZ}};
        FusionVector ma = {.axis = {magX, magY, magZ}};
        if (config.display.compass_orientation > meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_270) {
            ma = FusionRemap(ma, FusionRemapAlignmentNXNYPZ);
            ga = FusionRemap(ga, FusionRemapAlignmentNXNYPZ);
        }
        heading = FusionCompass(ga, ma, FusionConventionNed) + QMC6310_HEADING_OFFSET_DEG;
    } else {
        heading = atan2f(magY, magX) * RAD_TO_DEG + QMC6310_HEADING_OFFSET_DEG;
    }

    if (heading >= 360.0f)
        heading -= 360.0f;
    else if (heading < 0.0f)
        heading += 360.0f;
    heading = 360.0f - heading;
    if (heading >= 360.0f)
        heading -= 360.0f;

    heading = applyCompassOrientation(heading);
    if (screen)
        screen->setHeading(heading);
#endif

    return QMC6310_UPDATE_INTERVAL_MS;
}

void QMC6310Sensor::calibrate(uint16_t forSeconds)
{
#if !defined(MESHTASTIC_EXCLUDE_SCREEN)
    float xGauss = 0.0f;
    float yGauss = 0.0f;
    float zGauss = 0.0f;

    LOG_DEBUG("QMC6310 calibration started for %is", forSeconds);
    if (mag.isDataReady() && mag.readData() >= 0) {
        xGauss = mag.getX();
        yGauss = mag.getY();
        zGauss = mag.getZ();
        seedCalibrationExtrema(xGauss, yGauss, zGauss, highestX, lowestX, highestY, lowestY, highestZ, lowestZ);
    } else {
        seedCalibrationExtrema(0.0f, 0.0f, 0.0f, highestX, lowestX, highestY, lowestY, highestZ, lowestZ);
    }
    startCalibrationWindow(forSeconds);
#endif
}

#endif
