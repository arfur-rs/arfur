/*
 * Copyright (c) 2020-2021 REV Robotics
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

#include <optional>
#include <vector>

#include <frc/util/Color.h>

namespace rev {

/**
 * REV Robotics Color Sensor V3.
 *
 * This class allows access to a REV Robotics color sensor V3 on an I2C bus.
 */
class ColorMatch {
public:
    ColorMatch();

    /**
     * Add color to match object
     *
     * @param color color to add to matching
     *
     */
    void AddColorMatch(const frc::Color& color);

    /**
     * Set the confidence interval for determining color. Defaults to 0.95
     *
     * @param confidence    A value between 0 and 1
     */
    void SetConfidenceThreshold(double confidence);

    /**
     * MatchColor uses euclidean distance to compare a given normalized RGB
     * vector against stored values
     *
     * @param colorToMatch color to compare against stored colors
     *
     * @return  Matched color if detected
     */
    std::optional<frc::Color> MatchColor(const frc::Color& colorToMatch);

    /**
     * MatchColor uses euclidean distance to compare a given normalized RGB
     * vector against stored values
     *
     * @param colorToMatch color to compare against stored colors
     *
     * @param confidence The confidence value for this match, this is
     * simply 1 - euclidean distance of the two color vectors
     *
     * @return  Matched color if detected
     */
    std::optional<frc::Color> MatchColor(const frc::Color& colorToMatch,
                                         double& confidence);

    /**
     * MatchColor uses euclidean distance to compare a given normalized RGB
     * vector against stored values
     *
     * @param colorToMatch color to compare against stored colors
     *
     * @param confidence The confidence value for this match, this is
     * simply 1 - euclidean distance of the two color vectors
     *
     * @return  Closest matching color
     */
    frc::Color MatchClosestColor(const frc::Color& colorToMatch,
                                 double& confidence);

private:
    std::vector<frc::Color> m_colorsToMatch;
    double m_confidenceLevel;
};

}  // namespace rev
