/*
 * Copyright (c) 2018-2022 REV Robotics
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

#include <wpi/deprecated.h>

#include "rev/MotorFeedbackSensor.h"
#include "rev/REVLibError.h"

namespace rev {

/**
 * @deprecated Use SparkMaxAnalogSensor instead.
 */
class WPI_DEPRECATED("use SparkMaxAnalogSensor instead") CANAnalog
    : public MotorFeedbackSensor {
    friend class SparkMaxAnalogSensor;

public:
    /**
     * Analog sensors have the ability to either be absolute or relative.
     * By default, GetAnalog() will return an absolute analog sensor, but
     * it can also be configured to be a relative sensor instead.
     *
     * @deprecated Use SparkMaxAnalogSensor::Mode instead
     */
    enum class WPI_DEPRECATED("Use SparkMaxAnalogSensor::Mode instead")
        AnalogMode {
            kAbsolute = 0,
            kRelative = 1
        };

    virtual ~CANAnalog() {}

    /**
     * Get the voltage of the analog sensor.
     *
     * @return Voltage of the sensor
     */
    virtual double GetVoltage() const = 0;

    /**
     * Get the position of the sensor. Returns value in the native unit
     * of 'volt' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Position of the sensor in volts
     */
    virtual double GetPosition() const = 0;

    /**
     * Get the velocity of the sensor. Returns value in the native units of
     * 'volts per second' by default, and can be changed by a
     * scale factor using setVelocityConversionFactor().
     *
     * @return Velocity of the sensor in volts per second
     */
    virtual double GetVelocity() const = 0;

    /**
     * Set the conversion factor for the position of the analog sensor.
     * By default, revolutions per volt is 1. Changing the position conversion
     * factor will also change the position units.
     *
     * @param factor The conversion factor which will be multiplied by volts
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetPositionConversionFactor(double factor) = 0;

    /**
     * Get the current conversion factor for the position of the analog
     * sensor.
     *
     * @return Analog position conversion factor
     */
    virtual double GetPositionConversionFactor() const = 0;

    /**
     * Set the conversion factor for the velocity of the analog sensor.
     * By default, revolutions per volt second is 1. Changing the velocity
     * conversion factor will also change the velocity units.
     *
     * @param factor The conversion factor which will be multipled by volts per
     * second
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetVelocityConversionFactor(double factor) = 0;

    /**
     * Get the current conversion factor for the velocity of the analog
     * sensor.
     *
     * @return Analog velocity conversion factor
     */
    virtual double GetVelocityConversionFactor() const = 0;

    /**
     * Set the number of samples in the average for velocity readings. This
     * can be any value from 1 to 64.
     *
     * When the SparkMax controller is in Brushless mode, this
     * will not change any behavior.
     *
     * @param depth The average sampling depth between 1 and 64 (default)
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetAverageDepth(uint32_t depth) = 0;

    /**
     * Set the measurement period for velocity calculations.
     *
     * The basic formula to calculate velocity is change in position / change in
     * time. This parameter sets the change in time for measurement.
     *
     * @param period_ms Measurement period in milliseconds. This number may be
     * between 1 and 100 (default).
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetMeasurementPeriod(uint32_t period_ms) = 0;

    /**
     * Get the number of samples included in the average for velocity readings.
     *
     * @return The average sampling depth
     */
    virtual uint32_t GetAverageDepth() const = 0;

    /**
     * Get the measurement period used for velocity calculation.
     *
     * @return Measurement period in microseconds
     */
    virtual uint32_t GetMeasurementPeriod() const = 0;

    /**
     * Set the phase of the MotorFeedbackSensor so that it is set to be in
     * phase with the motor itself. This only works for quadrature
     * encoders. This will throw an error if the user tries to set
     * inverted while the SparkMax is Brushless and using the hall
     * effect sensor.
     *
     * @param inverted The phase of the encoder
     *
     * @return REVLibError::kOk if successful
     */
    // Already marked as virtual in MotorFeedbackSensor
    REVLibError SetInverted(bool inverted) override = 0;

    /**
     * Get the phase of the MotorFeedbackSensor. This will just return false
     * if the user tries to get inverted while the SparkMax is
     * Brushless and using the hall effect sensor.
     *
     * @return The phase of the encoder
     */
    bool GetInverted() const override = 0;

private:
    CANAnalog() {}

    int GetSparkMaxFeedbackDeviceID() const override = 0;
};

}  // namespace rev
