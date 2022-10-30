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

#ifdef _MSC_VER
// Disable deprecation warnings for this file when using VS compiler
#pragma warning(disable : 4996)
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include "rev/CANDigitalInput.h"
#include "rev/REVLibError.h"

namespace rev {

class CANSparkMax;

class SparkMaxLimitSwitch : public CANDigitalInput {
    // Friend to allow construction
    friend class CANSparkMax;

public:
    /**
     * Represents whether the circuit is open or closed when the switch is not
     * being pressed
     */
    enum class Type { kNormallyOpen = 0, kNormallyClosed = 1 };

    SparkMaxLimitSwitch(SparkMaxLimitSwitch&&) = default;
    SparkMaxLimitSwitch& operator=(SparkMaxLimitSwitch&&) = default;

    SparkMaxLimitSwitch(const SparkMaxLimitSwitch&) = default;

    ~SparkMaxLimitSwitch() override = default;

    /**
     * Get the state of the limit switch, whether or not it is enabled
     * (limiting the rotation of the motor).
     */
    bool Get() const override;

    /**
     * Enables or disables controller shutdown based on limit switch.
     */
    REVLibError EnableLimitSwitch(bool enable) override;

    /**
     * Returns true if limit switch is enabled.
     */
    bool IsLimitSwitchEnabled() const override;

private:
    enum class Direction { kForward = 0, kReverse = 1 };

    CANSparkMax* m_device;
    Direction m_direction;

    explicit SparkMaxLimitSwitch(CANSparkMax& device, Direction direction,
                                 Type switchType);
};

}  // namespace rev

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
