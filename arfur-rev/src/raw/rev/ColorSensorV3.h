/*
 * Copyright (c) 2020-2022 REV Robotics
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

#include <frc/I2C.h>
#include <frc/util/Color.h>
#include <hal/SimDevice.h>

#include "CIEColor.h"

namespace rev {

/**
 * REV Robotics Color Sensor V3.
 *
 * This class allows access to a REV Robotics color sensor V3 on an I2C bus.
 */
class ColorSensorV3 {
public:
    enum class GainFactor { k1x = 0, k3x = 1, k6x = 2, k9x = 3, k18x = 4 };

    enum class LEDPulseFrequency {
        k60kHz = 0x18,
        k70kHz = 0x40,
        k80kHz = 0x28,
        k90kHz = 0x30,
        k100kHz = 0x38,
    };

    enum class LEDCurrent {
        kPulse2mA = 0,
        kPulse5mA = 1,
        kPulse10mA = 2,
        kPulse25mA = 3,
        kPulse50mA = 4,
        kPulse75mA = 5,
        kPulse100mA = 6,
        kPulse125mA = 7,
    };

    enum class ProximityResolution {
        k8bit = 0x00,
        k9bit = 0x08,
        k10bit = 0x10,
        k11bit = 0x18,
    };

    enum class ProximityMeasurementRate {
        k6ms = 1,
        k12ms = 2,
        k25ms = 3,
        k50ms = 4,
        k100ms = 5,
        k200ms = 6,
        k400ms = 7,
    };

    enum class ColorResolution {
        k20bit = 0x00,
        k19bit = 0x10,
        k18bit = 0x20,
        k17bit = 0x30,
        k16bit = 0x40,
        k13bit = 0x50,
    };

    enum class ColorMeasurementRate {
        k25ms = 0,
        k50ms = 1,
        k100ms = 2,
        k200ms = 3,
        k500ms = 4,
        k1000ms = 5,
        k2000ms = 7,
    };

    struct RawColor {
        uint32_t red;
        uint32_t green;
        uint32_t blue;
        uint32_t ir;
        RawColor(uint32_t r, uint32_t g, uint32_t b, uint32_t _ir)
            : red(r), green(g), blue(b), ir(_ir) {}
    };

    /**
     * Constructs a ColorSensorV3.
     *
     * Note that the REV Color Sensor is really two devices in one package:
     * a color sensor providing red, green, blue and IR values, and a proximity
     * sensor.
     *
     * @param port  The I2C port the color sensor is attached to
     */
    explicit ColorSensorV3(frc::I2C::Port port);

    ColorSensorV3(ColorSensorV3&&) = default;
    ColorSensorV3& operator=(ColorSensorV3&&) = default;

    /**
     * Get the normalized RGB color from the sensor (normalized based on
     * total R + G + B)
     *
     * @return  frc::Color class with normalized sRGB values
     */
    frc::Color GetColor();

    /**
     * Get the raw color value from the sensor.
     *
     * @return  Raw color values from sensopr
     */
    RawColor GetRawColor();

    /**
     * Get the color converted to CIE XYZ color space using factory
     * calibrated constants.
     *
     * https://en.wikipedia.org/wiki/CIE_1931_color_space
     *
     * @return  CIEColor value from sensor
     */
    rev::CIEColor GetCIEColor();

    /**
     * Get the normalzied IR value from the sensor. Works best when within 2
     * inches and perpendicular to surface of interest.
     *
     * @return  Color class with normalized values
     */
    double GetIR();

    /**
     * Get the raw proximity value from the sensor ADC. This value is largest
     * when an object is close to the sensor and smallest when
     * far away.
     *
     * @return  Proximity measurement value, ranging from 0 to 2047 in
     *          default configuration
     */
    uint32_t GetProximity();

    /**
     * Set the gain factor applied to color ADC measurements.
     *
     * By default, the gain is set to 3x.
     *
     * @param gain  Gain factor applied to color ADC measurements
     *              measurements
     */
    void SetGain(GainFactor gain);

    /**
     * Configure the the IR LED used by the proximity sensor.
     *
     * These settings are only needed for advanced users, the defaults
     * will work fine for most teams. Consult the APDS-9151 for more
     * information on these configuration settings and how they will affect
     * proximity sensor measurements.
     *
     * @param freq      The pulse modulation frequency for the proximity
     *                  sensor LED
     * @param curr      The pulse current for the proximity sensor LED
     * @param pulses    The number of pulses per measurement of the
     *                  proximity sensor LED
     */
    void ConfigureProximitySensorLED(LEDPulseFrequency freq, LEDCurrent current,
                                     uint8_t pulses);

    /**
     * Configure the proximity sensor.
     *
     * These settings are only needed for advanced users, the defaults
     * will work fine for most teams. Consult the APDS-9151 for more
     * information on these configuration settings and how they will affect
     * proximity sensor measurements.
     *
     * @param res   Bit resolution output by the proximity sensor ADC.
     * @param rate  Measurement rate of the proximity sensor
     */
    void ConfigureProximitySensor(ProximityResolution res,
                                  ProximityMeasurementRate rate);

    /**
     * Configure the color sensor.
     *
     * These settings are only needed for advanced users, the defaults
     * will work fine for most teams. Consult the APDS-9151 for more
     * information on these configuration settings and how they will affect
     * color sensor measurements.
     *
     * @param res   Bit resolution output by the respective light sensor ADCs
     * @param rate  Measurement rate of the light sensor
     */
    void ConfigureColorSensor(ColorResolution res, ColorMeasurementRate rate);

    /**
     * Indicates if the device reset. Based on the power on status flag in the
     * status register. Per the datasheet:
     *
     * Part went through a power-up event, either because the part was turned
     * on or because there was power supply voltage disturbance (default at
     * first register read).
     *
     * This flag is self clearing
     *
     * @return  true if the device was reset
     */
    bool HasReset();

    /**
     * Indicates if the device can currently be communicated with.
     *
     * @return  true if the device is currently connected and responsive
     */
    bool IsConnected();

private:
    enum class Register {
        kMainCtrl = 0x00,
        kProximitySensorLED = 0x01,
        kProximitySensorPulses = 0x02,
        kProximitySensorRate = 0x03,
        kLightSensorMeasurementRate = 0x04,
        kLightSensorGain = 0x05,
        kPartID = 0x06,
        kMainStatus = 0x07,
        kProximityData = 0x08,
        kDataInfrared = 0x0A,
        kDataGreen = 0x0D,
        kDataBlue = 0x10,
        kDataRed = 0x13
    };

    enum class MainCtrlFields {
        kProximitySensorEnable = 0x01,
        kLightSensorEnable = 0x02,
        kRGBMode = 0x04
    };

    struct MainStatus {
        uint8_t PSDataStatus : 1;
        uint8_t PSInterruptStatus : 1;
        uint8_t PSLogicStatus : 1;
        uint8_t LSDataStatus : 1;
        uint8_t LSInterruptStatus : 1;
        uint8_t PowerOnStatus : 1;
        uint8_t : 2;
    };

    bool Write(Register reg, uint8_t data) {
        return m_i2c.Write(static_cast<uint8_t>(reg), data);
    }

    bool Read(Register reg, int count, uint8_t* data) {
        return m_i2c.Read(static_cast<uint8_t>(reg), count, data);
    }

    uint32_t To20Bit(uint8_t* val) {
        return (static_cast<uint32_t>(val[0]) |
                (static_cast<uint32_t>(val[1]) << 8) |
                (static_cast<uint32_t>(val[2]) << 16)) &
               0x03FFFF;
    }

    uint16_t To11Bit(uint8_t* val) {
        return (static_cast<uint16_t>(val[0]) |
                (static_cast<uint16_t>(val[1]) << 8)) &
               0x7FF;
    }

    uint32_t Read20BitRegister(Register reg);
    uint16_t Read11BitRegister(Register reg);

    bool CheckDeviceID(bool reportErrors);
    void InitializeDevice();
    MainStatus GetStatus();

    static const double Cmatrix[9];

    frc::I2C m_i2c;

    hal::SimDevice m_simDevice;
    hal::SimDouble m_simR, m_simG, m_simB, m_simIR, m_simProx;
};

}  // namespace rev
