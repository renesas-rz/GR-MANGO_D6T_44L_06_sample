// SPDX-License-Identifier: MIT
/*
 * MIT License
 * Copyright (c) 2019, 2018 - present OMRON Corporation
 * Copyright (c) 2019 Renesas Electronics Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef D6T_44L_06_H
#define D6T_44L_06_H

#include "mbed.h"

/** xxxxxxxxxxxxxx [D6T_44L_06] class
 *
 * @note Synchronization level: Thread safe
 *
 * Example of how to get data from this sensor:
 * @code
 *
 * #include "mbed.h"
 * #include "D6T_44L_06.h"
 *
 * D6T_44L_06 d6taaaa(I2C_SDA, I2C_SCL);
 *
 * int main() {
 *     uint16_t illm;
 * 
 *     opt3001.setup();
 *     ThisThread::sleep_for(110);
 * 
 *     while(1) {
 *         opt3001.read(&illm);
 *         printf("llluminance : %5.2f [lx]\r\n", illm / 100.0);
 *         ThisThread::sleep_for(200);
 *     }
 * }
 * @endcode
 */
class D6T_44L_06
{
public:
    /** Create a sensor instance
     *  
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     */
    D6T_44L_06(PinName sda, PinName scl);

    /** Initialize a sensor device
     *
     *  Should be called once per lifetime of the object.
     *  Please wait about 100ms before calling the read function.
     *  @return true on success, false on failure
     */
    bool setup(void);

    /** Read the current data from sensor
     *
     *  @return true on success, false on failure
     */
    bool read(int16_t* ptat, int16_t* buf);

private:
    I2C mI2c_;
    int mAddr;


    uint8_t calc_crc(uint8_t data);
    bool D6T_checkPEC(uint8_t buf[], int n);
    int16_t conv8us_s16_le(uint8_t* buf, int n);
    int read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len);
};

#endif
