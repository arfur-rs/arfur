/**
 * Basic compatibility code for Rust WPILib <-> C++ WPILib.
 *
 * Ideally, this file wouldn't exist, and `cxx` would be smart enough to handle
 * things such as constructor generation for us, but as of the time of writing,
 * it does not. This file may be replaced as cxx issues such as
 * [280](https://github.com/dtolnay/cxx/issues/280) are resolved.
 */

#include "frc/ADXRS450_Gyro.h"

namespace frc {
  std::unique_ptr<ADXRS450_Gyro> new_ADXRS450_Gyro() {
    return std::unique_ptr<ADXRS450_gyro>(new ADXRS450_Gyro());
  }
}
