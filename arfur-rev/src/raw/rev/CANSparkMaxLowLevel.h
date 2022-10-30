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

#include <string>
#include <vector>

#include <frc/motorcontrol/MotorController.h>
#include <hal/Types.h>
#include <wpi/deprecated.h>

#include "rev/ControlType.h"
#include "rev/REVLibError.h"

// Defined in HIL tester source
class ConfigBase;

namespace rev {

class CANSparkMax;

class CANSparkMaxLowLevel : public frc::MotorController {
    friend class CANAnalog;
    friend class CANDigitalInput;
    friend class CANEncoder;
    friend class CANPIDController;
    friend class CANSensor;
    friend class CANSparkMax;
    friend class MotorFeedbackSensor;
    friend class SparkMaxAnalogSensor;
    friend class SparkMaxAlternateEncoder;
    friend class SparkMaxLimitSwitch;
    friend class SparkMaxPIDController;
    friend class SparkMaxRelativeEncoder;

    // Defined in HIL tester source
    friend class ::ConfigBase;

public:
    static const uint8_t kAPIMajorVersion;
    static const uint8_t kAPIMinorVersion;
    static const uint8_t kAPIBuildVersion;
    static const uint32_t kAPIVersion;

    enum class MotorType { kBrushed = 0, kBrushless = 1 };

    enum class ControlType {
        kDutyCycle = 0,
        kVelocity = 1,
        kVoltage = 2,
        kPosition = 3,
        kSmartMotion = 4,
        kCurrent = 5,
        kSmartVelocity = 6
    };

    enum class ParameterStatus {
        kOK = 0,
        kInvalidID = 1,
        kMismatchType = 2,
        kAccessMode = 3,
        kInvalid = 4,
        kNotImplementedDeprecated = 5,
    };

    enum class PeriodicFrame {
        kStatus0 = 0,
        kStatus1 = 1,
        kStatus2 = 2,
        kStatus3 = 3
    };

    struct PeriodicStatus0 {
        double appliedOutput;
        uint16_t faults;
        uint16_t stickyFaults;
        MotorType motorType;
        bool isFollower;
        uint8_t lock;
        uint8_t roboRIO;
        uint8_t isInverted;
        uint64_t timestamp;
    };

    struct PeriodicStatus1 {
        double sensorVelocity;
        uint8_t motorTemperature;
        double busVoltage;
        double outputCurrent;
        uint64_t timestamp;
    };

    struct PeriodicStatus2 {
        double sensorPosition;
        double iAccum;
        uint64_t timestamp;
    };

    enum class TelemetryID {
        kBusVoltage = 0,
        kOutputCurrent,
        kVelocity,
        kPosition,
        kIAccum,
        kAppliedOutput,
        kMotorTemp,
        kFaults,
        kStickyFaults,
        kAnalogVoltage,
        kAnalogPosition,
        kAnalogVelocity,
        kAltEncPosition,
        kAltEncVelocity,
        kTotalStreams
    };

    struct TelemetryMessage {
        TelemetryID id;
        float value = 0;
        uint64_t timestamp = 0;
        const char* name;
        const char* units;
        float lowerBnd;
        float upperBnd;
    };

    /**
     * Closes the SPARK MAX Controller
     */
    virtual ~CANSparkMaxLowLevel();

    /**
     * Get the firmware version of the SPARK MAX.
     *
     * @return uint32_t Firmware version integer. Value is represented as 4
     * bytes, Major.Minor.Build H.Build L
     *
     */
    uint32_t GetFirmwareVersion();

    uint32_t GetFirmwareVersion(bool& isDebugBuild);

    /**
     * Get the firmware version of the SPARK MAX as a string.
     *
     * @return std::string Human readable firmware version string
     *
     */
    std::string GetFirmwareString();

    /**
     * Get the unique serial number of the SPARK MAX. Currently not implemented.
     *
     * @return std::vector<uint8_t> Vector of bytes representig the unique
     * serial number
     *
     */
    std::vector<uint8_t> GetSerialNumber();

    /**
     * Get the configured Device ID of the SPARK MAX.
     *
     * @return int device ID
     *
     */
    int GetDeviceId() const;

    /**
     * Get the motor type setting from when the SparkMax was created.
     *
     * This does not use the Get Parameter API which means it does not read
     * what motor type is stored on the SparkMax itself. Instead, it reads
     * the stored motor type from when the SparkMax object was first created.
     *
     * @return MotorType Motor type setting
     *
     * @deprecated Use GetMotorType() instead
     */
    WPI_DEPRECATED("Use GetMotorType() instead")
    MotorType GetInitialMotorType();

    /**
     * Get the motor type setting for the SPARK MAX.
     *
     * @return MotorType Motor type setting
     *
     */
    MotorType GetMotorType();

    /**
     * Set the rate of transmission for periodic frames from the SPARK MAX
     *
     * Each motor controller sends back three status frames with different
     * data at set rates. Use this function to change the default rates.
     *
     * Defaults:
     * Status0 - 10ms
     * Status1 - 20ms
     * Status2 - 50ms
     *
     * This value is not stored in the FLASH after calling burnFlash()
     * and is reset on powerup.
     *
     * Refer to the SPARK MAX reference manual on details for how and when
     * to configure this parameter.
     *
     * @param frame Which periodic frame to change the period of
     * @param periodMs The rate the controller sends the frame to the
     * controller.
     *
     * @return REVLibError::kOk if successful
     *
     */
    REVLibError SetPeriodicFramePeriod(PeriodicFrame frame, int periodMs);

    /**
     * Set the control frame send period for the native CAN Send thread. To
     * disable periodic sends, set periodMs to 0.
     *
     * @param periodMs The send period in milliseconds between 1ms and 100ms
     * or set to 0 to disable periodic sends. Note this is not updated until
     * the next call to Set() or SetReference().
     *
     */
    void SetControlFramePeriodMs(int periodMs);

    /**
     * Restore motor controller parameters to factory default
     *
     * @param persist If true, burn the flash with the factory default
     * parameters
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError RestoreFactoryDefaults(bool persist = false);

    /**
     * Allow external controllers to recieve control commands over USB.
     * For example, a configuration where the heartbeat (and enable/disable)
     * is sent by the main controller, but control frames are sent by
     * other CAN devices over USB.
     *
     * This is global for all controllers on the same bus.
     *
     * This does not disable sending control frames from this device. To prevent
     * conflicts, do not enable this feature and also send Set() for
     * SetReference() from the controllers you wish to control.
     *
     * @param enable Enable or disable external control
     *
     */
    static void EnableExternalUSBControl(bool enable);

#ifndef __FRC_ROBORIO__
    /**
     * Send enabled or disabled command to controllers. This is global for all
     * controllers on the same bus, and will only work for non-roboRIO targets
     * in non-competiton use. This function will also not work if a roboRIO is
     * present on the CAN bus.
     *
     * This does not disable sending control frames from this device. To prevent
     * conflicts, do not enable this feature and also send Set() for
     * SetReference() from the controllers you wish to control.
     *
     * @param enable Enable or disable external control
     *
     */
    static void SetEnable(bool enable);
#endif

protected:
    enum class FeedbackSensorType {
        kNoSensor = 0,
        kHallSensor = 1,
        kQuadrature = 2,
        kSensorless = 3,
        kAnalog = 4,
        kAltQuadrature = 5,
    };

    struct FollowConfigBits {
        uint32_t rsvd1 : 18;
        uint32_t invert : 1;
        uint32_t rsvd2 : 5;
        uint32_t predefined : 8;
    };

    struct FollowConfig {
        uint32_t leaderArbId;
        union FollowConfigUnion {
            uint32_t value;
            FollowConfigBits bits;
        } config;
    };

    PeriodicStatus0 GetPeriodicStatus0();
    PeriodicStatus1 GetPeriodicStatus1();
    PeriodicStatus2 GetPeriodicStatus2();

    REVLibError SetFollow(FollowConfig config);

    REVLibError SetpointCommand(
        double value,
        CANSparkMaxLowLevel::ControlType ctrl = ControlType::kDutyCycle,
        int pidSlot = 0, double arbFeedforward = 0, int arbFFUnits = 0);

    float GetSafeFloat(float f);

    MotorType m_motorType;
    // The type is void* because we don't want to expose c_SparkMax_handle to
    // the consumers of this header file
    void* m_sparkMaxHandle;

private:
    explicit CANSparkMaxLowLevel(int deviceID, MotorType type);

    int m_deviceID;
};

}  // namespace rev
