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

#include <frc/util/Color.h>

namespace rev {

class CIEColor {
public:
    CIEColor(double X, double Y, double Z) : X(X), Y(Y), Z(Z), mag(X + Y + Z) {}

    /**
     * Get the X component of the color
     *
     * @return  CIE X
     */
    double GetX() { return X; }

    /**
     * Get the Y component of the color
     *
     * @return  CIE Y
     */
    double GetY() { return Y; }

    /**
     * Get the Z component of the color
     *
     * @return  CIE Z
     */
    double GetZ() { return Z; }

    /**
     * Get the x calculated coordinate
     * of the CIE 19313 color space
     *
     * https://en.wikipedia.org/wiki/CIE_1931_color_space
     *
     * @return  CIE Yx
     */
    double GetYx() { return X / mag; }

    /**
     * Get the y calculated coordinate
     * of the CIE 19313 color space
     *
     * https://en.wikipedia.org/wiki/CIE_1931_color_space
     *
     * @return  CIE Yy
     */
    double GetYy() { return Y / mag; }

private:
    // This is private until we get it working correctly
    frc::Color ToRGB();
    static const double IlluminantD65[3];
    static const double XYZtoRGB[9];
    double X;
    double Y;
    double Z;
    double mag;
};

}  // namespace rev
