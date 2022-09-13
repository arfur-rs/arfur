#pragma once

#include "frc/ADXRS450_Gyro"

inline ADXRS450_Gyro create_adxrs450_gyro() {
  return new ADXRS450_Gyro();
}
