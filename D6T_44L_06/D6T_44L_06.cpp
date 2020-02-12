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

#include "D6T_44L_06.h"

#define D6T_ADDR (0x0A << 1)  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T_44L_06-44L-06/06H, D6T_44L_06-8L-09/09H, for D6T_44L_06-1A-01/02

#define N_ROW 4
#define N_PIXEL (4 * 4)

#define N_READ ((N_PIXEL + 1) * 2 + 1)

// D6T_44L_06 implementation
D6T_44L_06::D6T_44L_06(PinName sda, PinName scl) :
     mI2c_(sda, scl)
{
    mAddr = D6T_ADDR;
    mI2c_.frequency(100000);
}

bool D6T_44L_06::setup(void)
{
    return true;
}

bool D6T_44L_06::read(int16_t* ptat, int16_t* buf)
{
    int ret;
    uint8_t wk_buf[N_READ];
    int i;
    int j;

    ret = read_reg(D6T_CMD, wk_buf, N_READ);
    if (ret != 0) {
        return false;
    }

    if (D6T_checkPEC(wk_buf, N_READ - 1)) {
        return false;
    }

    // 1st data is PTAT measurement (: Proportional To Absolute Temperature)
    if (ptat != NULL) {
        *ptat = conv8us_s16_le(wk_buf, 0);
    }

    // loop temperature pixels of each thrmopiles measurements
    if (buf != NULL) {
        for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
            buf[i] = conv8us_s16_le(wk_buf, j);
        }
    }

    return true;
}

uint8_t D6T_44L_06::calc_crc(uint8_t data)
{
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

bool D6T_44L_06::D6T_checkPEC(uint8_t buf[], int n)
{
    uint8_t crc;
    int i;

    crc = calc_crc(mAddr);
    crc = calc_crc(D6T_CMD ^ crc);
    crc = calc_crc((mAddr | 1) ^ crc);
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    return crc != buf[n];
}

int16_t D6T_44L_06::conv8us_s16_le(uint8_t* buf, int n)
{
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}


int D6T_44L_06::read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len)
{
    int ret;

    mI2c_.lock();
    ret = mI2c_.write(mAddr, (char *)&reg, 1, true);
    if (ret == 0) {
        ret = mI2c_.read(mAddr, (char *)pbuf, len);
    }
    mI2c_.unlock();

    return ret;
}

