/*
 * Copyright (c) 2021 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>

#ifdef _MSC_VER
// Disable deprecation warnings for this file when using VS compiler
#pragma warning(disable : 4996)
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include "rev/CANSensor.h"
#include "rev/REVLibError.h"

namespace rev {

/** A sensor that can be used to provide rotational feedback to a motor
 * controller */
class MotorFeedbackSensor : public CANSensor {
    friend class CANAnalog;
    friend class CANEncoder;

public:
    virtual ~MotorFeedbackSensor() {}

    /**
     * Set the phase of the MotorFeedbackSensor so that it is set to be in phase
     * with the motor itself. This only works for quadrature encoders and analog
     * sensors. This will throw an error if the user tries to set the inversion
     * of the hall sensor.
     *
     * @param inverted The phase of the sensor
     * @return REVLibError::kOk if successful
     */
    // TODO(Noah): Mark as virtual when CANSensor is deleted
    REVLibError SetInverted(bool inverted) override = 0;

    /**
     * Get the phase of the MotorFeedbackSensor. This will just return false if
     * the user tries to get the inversion of the hall sensor.
     *
     * @return The phase of the sensor
     */
    // TODO(Noah): Mark as virtual when CANSensor is deleted
    bool GetInverted() const override = 0;

private:
    MotorFeedbackSensor() : CANSensor() {}

    // TODO(Noah): Mark as virtual when CANSensor is deleted
    int GetSparkMaxFeedbackDeviceID() const override = 0;
};

}  // namespace rev

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
