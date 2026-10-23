// Drivers layer.
//
// One driver module per physical device, built on top of the HAL layer:
//
//   - IMU driver (ICM-45686, 6-DOF accel + gyro)   -- ImuIcm45686.h  [done]
//   - Quadrature encoder capture (left/right)      -- Encoders.h     [done]
//   - Motor driver (TB6612FNG dual H-bridge)       -- Motors.h       [done]
//   - FRAM driver (MB85RS4MTY, 512 KB, SPI)        -- Fram.h         [done]
//   - Sensor sub-processor link (wall packets from Serial2) -- Sensors.h [done]
//   - ESP32 radio link (frames log/config data for Serial5)
//
// See firmware_architecture_adr.md for the intended layering.

#pragma once

#include "Encoders.h"
#include "Fram.h"
#include "ImuIcm45686.h"
#include "Motors.h"
#include "Sensors.h"
