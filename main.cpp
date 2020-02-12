// SPDX-License-Identifier: MIT
/*
 * MIT License
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

#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "r_dk2_if.h"
#include "r_drp_simple_isp.h"
#include "D6T_44L_06.h"
#include "dcache-control.h"
#include "AsciiFont.h"

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         (640)    /* VGA */
#define VIDEO_PIXEL_VW         (480)    /* VGA */

#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * 1) + 63u) & ~63u)
#define FRAME_BUFFER_STRIDE_2  (((VIDEO_PIXEL_HW * 2) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)

#define DRP_FLG_TILE_ALL       (R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5)
#define DRP_FLG_CAMER_IN       (0x00000100)

/* ASCII BUFFER Parameter GRAPHICS_LAYER_3 */
#define ASCII_BUFFER_BYTE_PER_PIXEL   (2)
#define ASCII_BUFFER_STRIDE           (((VIDEO_PIXEL_HW * ASCII_BUFFER_BYTE_PER_PIXEL) + 31u) & ~31u)
#define ASCII_COLOR_WHITE             (0xFFFF)
#define ASCII_COLOR_BLACK             (0x00F0)
#define ASCII_FONT_SIZE               (3)

#define TILE_ALPHA_MAX      (0x0F)
#define TILE_ALPHA_SWITCH2  (0x0A)
#define TILE_ALPHA_SWITCH1  (0x06)
#define TILE_ALPHA_DEFAULT  (0x03)

#define TILE_TEMP_MARGIN_UPPER (20)
#define TILE_TEMP_MARGIN_UNDER (70)

#define TILE_RESO_4         (4)
#define TILE_RESO_8         (8)
#define TILE_RESO_16        (16)
#define TILE_RESO_32        (32)
#define TILE_RESO_60        (60)
#define TILE_RESO_64        (64)
#define TILE_RESO_120       (120)
#define TILE_RESO_160       (160)
#define TILE_RESO_240       (240)
#define TILE_RESO_320       (320)

#define TILE_SIZE_HW_4x4    (VIDEO_PIXEL_HW/TILE_RESO_4)
#define TILE_SIZE_VW_4x4    (VIDEO_PIXEL_VW/TILE_RESO_4)
#define TILE_SIZE_HW_8x8    (VIDEO_PIXEL_HW/TILE_RESO_8)
#define TILE_SIZE_VW_8x8    (VIDEO_PIXEL_VW/TILE_RESO_8)
#define TILE_SIZE_HW_16x16  (VIDEO_PIXEL_HW/TILE_RESO_16)
#define TILE_SIZE_VW_16x16  (VIDEO_PIXEL_VW/TILE_RESO_16)
#define TILE_SIZE_HW_32x32  (VIDEO_PIXEL_HW/TILE_RESO_32)
#define TILE_SIZE_VW_32x32  (VIDEO_PIXEL_VW/TILE_RESO_32)
#define TILE_SIZE_HW_64x60  (VIDEO_PIXEL_HW/TILE_RESO_64)
#define TILE_SIZE_VW_64x60  (VIDEO_PIXEL_VW/TILE_RESO_60)
#define TILE_SIZE_HW_160x120  (VIDEO_PIXEL_HW/TILE_RESO_160)
#define TILE_SIZE_VW_160x120  (VIDEO_PIXEL_VW/TILE_RESO_120)
#define TILE_SIZE_HW_320x240  (VIDEO_PIXEL_HW/TILE_RESO_320)
#define TILE_SIZE_VW_320x240  (VIDEO_PIXEL_VW/TILE_RESO_240)

#define SUB_PHASE_MAX       (10)
#define SUB_PHASE_DEMO1     SUB_PHASE_MAX*2
#define SUB_PHASE_DEMO2     SUB_PHASE_MAX*3
#define PHASE_DELAY         (200)


static DisplayBase Display;

static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_yuv[FRAME_BUFFER_STRIDE_2 * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_ascii0[ASCII_BUFFER_STRIDE * VIDEO_PIXEL_VW]__attribute((aligned(32)));
static uint8_t fbuf_ascii1[ASCII_BUFFER_STRIDE * VIDEO_PIXEL_VW]__attribute((aligned(32)));

AsciiFont* p_af0;
AsciiFont* p_af1;
int screen = 0;

/* thermal data array[y][x] */
static float array4x4[TILE_RESO_4][TILE_RESO_4];
static float array8x8[TILE_RESO_8][TILE_RESO_8];
static float array16x16[TILE_RESO_16][TILE_RESO_16];
static float array32x32[TILE_RESO_32][TILE_RESO_32];
static float array64x60[TILE_RESO_60][TILE_RESO_64];
static float array160x120[TILE_RESO_120][TILE_RESO_160];

static r_drp_simple_isp_t param_isp __attribute((section("NC_BSS")));
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static Thread drpTask(osPriorityHigh, 1024*8);
static D6T_44L_06 d6t_44l(I2C_SDA, I2C_SCL);

/*******************************************************************************
* Function Name: normalize0to1
* Description  : Normalize thermal data of a reception packet to the range of 0-1.
*                Also the value outside min, max is saturated.
* Arguments    : data     - input tempetue data
*                min      - smallest threshold tempature value
*                           (The integer which set a centigrade to 10 times)
*                max      - highest threshold  tempature value
*                           (The integer which set a centigrade to 10 times)
* Return Value : normalized output data
*******************************************************************************/
float normalize0to1(int16_t data, int min, int max)
{
    float normalized;

    if (data <= min) {
        normalized = 0.0;
    }
    else if (max <= data) {
        normalized = 1.0;
    }
    else {
        normalized = ((float)(data - min)) / ((float)(max - min));
    }
    return normalized;
}
/*******************************************************************************
 End of function normalize0to1
*******************************************************************************/

/*******************************************************************************
* Function Name: conv_normalize_to_color
* Description  : Change a floating-point data of 0.0-1.0 to the pixel color of
*                ARGB4444(truth order:GBAR).
*                Low temperature changes blue and high temperature to red.
* Arguments    : data        - Normalized thermal data from 0.0 to 1.0.
* Return Value : ARGB4444 pixel color data.
*******************************************************************************/
static uint16_t conv_normalize_to_color(uint8_t alpha, float data) {
    uint8_t green;
    uint8_t blue;
    uint8_t red;

    if (0.0 == data) {
        /* Display blue when the temperature is below the minimum. */
        blue  = 0x0F;
        green = 0x00;
        red   = 0x00;
    }
    else if (1.0 == data) {
        /* Display red when the maximum temperature is exceeded. */
        blue  = 0x00;
        green = 0x00;
        red   = 0x0F;
    }
    else {
        float cosval   = cos( 4 * M_PI * data);
        int    color    = (int)((((-cosval)/2) + 0.5) * 15);
        if (data < 0.25) {
            blue  = 0xF;
            green = color;
            red   = 0x00;
        }
        else if (data < 0.50) {
            blue  = color;
            green = 0x0F;
            red   = 0x00;
        }
        else if (data < 0.75) {
            blue  = 0x00;
            green = 0x0F;
            red   = color;
        }
        else {
            blue  = 0x00;
            green = color;
            red   = 0x0F;
        }
    }

    return ((green << 12) | (blue << 8) | (alpha << 4) | red);
}

/*******************************************************************************
 End of function conv_normalize_to_color
*******************************************************************************/

/*******************************************************************************
* Function Name: liner_interpolation
* Description  : Expand float data array from array[y_in_size][x_in_size] to array[y_out_size][x_out_size]
*                Linear complementation is used for expansion algorithm.
* Arguments    : p_in_array   - pointer of input data
*                p_out_array  - pointer of output data
*                x_in_size    - input array x size
*                y_in_size    - input array y size
*                x_out_size   - output array x size
*                y_out_size   - output array y size
* Return Value : none
*******************************************************************************/
static void liner_interpolation(float* p_in_array, float* p_out_array, int x_in_size, int y_in_size, int x_out_size, int y_out_size)
{
    int   x_in;
    int   y_in;
    int   x_out_start;
    int   x_out_goal;
    float x_delta;
    int   x_w_pos;
    int   y_out_start;
    int   y_out_goal;
    float y_delta;
    int   y_w_pos;
    float data_start;
    float data_goal;
    float data_w_pos;

    /* expand x direction */
    for (y_in = 0; y_in < y_in_size; y_in++) {
        y_out_goal = (y_out_size - 1) * y_in / (y_in_size - 1);
        for (x_in = 1; x_in < x_in_size; x_in++) {
            x_out_start  = (x_out_size - 1) * (x_in - 1) / (x_in_size - 1);
            x_out_goal   = (x_out_size - 1) * (x_in    ) / (x_in_size - 1);
            x_delta   = x_out_goal - x_out_start;

            data_start = p_in_array[(x_in - 1) + (x_in_size * y_in)];
            data_goal  = p_in_array[(x_in    ) + (x_in_size * y_in)];

            for (x_w_pos = x_out_start; x_w_pos <= x_out_goal; x_w_pos++) {
                data_w_pos = (data_start * ((x_out_goal - x_w_pos ) / x_delta))
                           + (data_goal  * ((x_w_pos - x_out_start) / x_delta));
                p_out_array[x_w_pos + (x_out_size * y_out_goal)] = data_w_pos;
            }
        }
    }

    /* expand y direction */
    for (x_w_pos = 0; x_w_pos < x_out_size; x_w_pos++) {
        for (y_in = 1; y_in < y_in_size; y_in++) {
            y_out_start  = (y_out_size - 1) * (y_in - 1) / (y_in_size - 1);
            y_out_goal   = (y_out_size - 1) * (y_in    ) / (y_in_size - 1);
            y_delta   = y_out_goal - y_out_start;

            data_start = p_out_array[x_w_pos + (x_out_size * y_out_start)];
            data_goal  = p_out_array[x_w_pos + (x_out_size * y_out_goal)];

            for (y_w_pos = y_out_start+1; y_w_pos < y_out_goal; y_w_pos++) {
                data_w_pos = (data_start * ((y_out_goal - y_w_pos ) / y_delta))
                           + (data_goal  * ((y_w_pos - y_out_start) / y_delta));
                p_out_array[x_w_pos + (x_out_size * y_w_pos)] = data_w_pos;
            }
        }
    }
}
/*******************************************************************************
 End of function liner_interpolation
*******************************************************************************/

/*******************************************************************************
* Function Name: update_thermograph
* Description  : Update display thermograph.
* Arguments    : reso_x    - input array x size
*                reso_y    - input array y size
*                tile_hw   - tile width pixel size
*                tile_vw   - tile height pixel size
*                alpha     - alpha pixel value of thermograph
*                p_array   - pointer of thermal data array
*                title_str - title string
* Return Value : none
*******************************************************************************/
void update_thermograph(int reso_x, int reso_y, int tile_hw, int tile_vw, uint8_t alpha, float* p_array, const char* title_str)
{
    AsciiFont* p_af;
    uint8_t*   p_fbuf;
    int x, y;
    uint16_t color;

    if (0 == screen)
    {
        p_af = p_af0;
        p_fbuf = &fbuf_ascii0[0];
    }
    else
    {
        p_af = p_af1;
        p_fbuf = &fbuf_ascii1[0];
    }

    for (y = 0; y < reso_y; y++)
    {
        for (x = 0; x < reso_x; x++)
        {
            color = conv_normalize_to_color(alpha, p_array[(y * reso_x)  + x]);
            p_af->Erase(color, (tile_hw * x), (tile_vw * y), tile_hw, tile_vw);
        }
    }
    p_af->Erase(ASCII_COLOR_WHITE, 0, 0, 30, 20);
    p_af->DrawStr(title_str, 0, 0, ASCII_COLOR_BLACK, ASCII_FONT_SIZE, 18);

    dcache_clean(p_fbuf, sizeof(fbuf_ascii0));
    Display.Graphics_Read_Change(DisplayBase::GRAPHICS_LAYER_3, (void *)p_fbuf);

    if (0 == screen)
    {
        screen = 1;
    }
    else
    {
        screen = 0;
    }

    return;
}
/*******************************************************************************
 End of function update_thermograph
*******************************************************************************/

/*******************************************************************************
* Function Name: clear_thermograph
* Description  : Turn off the thermograph on the display.
* Arguments    : title_str - title string
* Return Value : none
*******************************************************************************/
void clear_thermograph(const char* title_str)
{
    AsciiFont* p_af;
    uint8_t*   p_fbuf;

    if (0 == screen)
    {
        p_af = p_af0;
        p_fbuf = &fbuf_ascii0[0];
    }
    else
    {
        p_af = p_af1;
        p_fbuf = &fbuf_ascii1[0];
    }

    p_af->Erase(0x0000);
    p_af->Erase(ASCII_COLOR_WHITE, 0, 0, 30, 20);
    p_af->DrawStr(title_str, 0, 0, ASCII_COLOR_BLACK, ASCII_FONT_SIZE, 10);
    dcache_clean(p_fbuf, sizeof(fbuf_ascii0));
    Display.Graphics_Read_Change(DisplayBase::GRAPHICS_LAYER_3, (void *)p_fbuf);

    if (0 == screen)
    {
        screen = 1;
    }
    else
    {
        screen = 0;
    }

    return;
}
/*******************************************************************************
 End of function clear_thermograph
*******************************************************************************/

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}

static void cb_drp_finish(uint8_t id) {
    uint32_t tile_no;
    uint32_t set_flgs = 0;

    // Change the operation state of the DRP library notified by the argument to finish
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++) {
        if (drp_lib_id[tile_no] == id) {
            set_flgs |= (1 << tile_no);
        }
    }
    drpTask.flags_set(set_flgs);
}

static void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

#if MBED_CONF_APP_LCD
static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_yuv,
        FRAME_BUFFER_STRIDE_2,
        DisplayBase::GRAPHICS_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    ThisThread::sleep_for(50);
    EasyAttach_LcdBacklight(true);
}
#endif

static void Start_Thermo_Display(void) {
    DisplayBase::rect_t rect;

    memset(fbuf_ascii1, 0, sizeof(fbuf_ascii1));

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_3,
        (void *)fbuf_ascii0,
        ASCII_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_ARGB4444,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_3);

}


static void drp_task(void) {
    EasyAttach_Init(Display);
    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, IntCallbackFunc_Vfield);
    Start_Video_Camera();
#if MBED_CONF_APP_LCD
    Start_LCD_Display();
#endif
    Start_Thermo_Display();

    R_DK2_Initialize();

    /* Load DRP Library                 */
    /*        +-----------------------+ */
    /* tile 0 |                       | */
    /*        +                       + */
    /* tile 1 |                       | */
    /*        +                       + */
    /* tile 2 |                       | */
    /*        + SimpleIsp bayer2yuv_6 + */
    /* tile 3 |                       | */
    /*        +                       + */
    /* tile 4 |                       | */
    /*        +                       + */
    /* tile 5 |                       | */
    /*        +-----------------------+ */
    R_DK2_Load(g_drp_lib_simple_isp_bayer2yuv_6,
               R_DK2_TILE_0,
               R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);

    memset(&param_isp, 0, sizeof(param_isp));
    param_isp.src    = (uint32_t)fbuf_bayer;
    param_isp.dst    = (uint32_t)fbuf_yuv;
    param_isp.width  = VIDEO_PIXEL_HW;
    param_isp.height = VIDEO_PIXEL_VW;
    param_isp.gain_r = 0x1800;
    param_isp.gain_g = 0x1000;
    param_isp.gain_b = 0x1C00;
    param_isp.bias_r = -16;
    param_isp.bias_g = -16;
    param_isp.bias_b = -16;

    while (true) {
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);

        // Start DRP and wait for completion
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    }
}

int main(void) {
    int16_t pdta;
    int16_t buf[16];
    int16_t phase = 0;
    int16_t sub_phase = 0;
    char    str[32];

    // Start DRP task
    drpTask.start(callback(drp_task));

    printf("\x1b[2J");  // Clear screen

    // setup sensors
    d6t_44l.setup();

    ThisThread::sleep_for(150);

    AsciiFont ascii_font0(fbuf_ascii0, VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, ASCII_BUFFER_STRIDE, ASCII_BUFFER_BYTE_PER_PIXEL);
    AsciiFont ascii_font1(fbuf_ascii1, VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, ASCII_BUFFER_STRIDE, ASCII_BUFFER_BYTE_PER_PIXEL);

    p_af0 = &ascii_font0;
    p_af1 = &ascii_font1;

    while (1) {
        int x, y;

        printf("\x1b[%d;%dH", 0, 0);  // Move cursor (y , x)

        while (d6t_44l.read(&pdta, &buf[0]) == false) {
            ThisThread::sleep_for(10);
        }

        printf("PTAT: %6.1f[degC]\r\n", pdta / 10.0);
        for (int i = 0; i < 16; i++) {
            printf("%4.1f, ", buf[i] / 10.0);

            if ((i % 4) == 4 - 1) {
                printf("\r\n");
            }
        }

        for (y = 0; y < TILE_RESO_4; y++)
        {
            for (x = 0; x < TILE_RESO_4; x++)
            {
                array4x4[y][x] = normalize0to1(buf[x + (TILE_RESO_4*y)], pdta - TILE_TEMP_MARGIN_UNDER,  pdta + TILE_TEMP_MARGIN_UPPER);
            }
        }

        switch (phase)
        {
            case 0:
                sprintf( str, "PTAT[%2.1f]   4*4  " , pdta/10.0 );
                update_thermograph(TILE_RESO_4, TILE_RESO_4, TILE_SIZE_HW_4x4, TILE_SIZE_VW_4x4, TILE_ALPHA_MAX, &array4x4[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_DEMO1)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 1:
                sprintf( str, "PTAT[%2.1f]   8*8  " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array8x8[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_8, TILE_RESO_8);
                update_thermograph(TILE_RESO_8, TILE_RESO_8, TILE_SIZE_HW_8x8, TILE_SIZE_VW_8x8, TILE_ALPHA_MAX, &array8x8[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 2:
                sprintf( str, "PTAT[%2.1f]  16*16 " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array16x16[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_16, TILE_RESO_16);
                update_thermograph(TILE_RESO_16, TILE_RESO_16, TILE_SIZE_HW_16x16, TILE_SIZE_VW_16x16, TILE_ALPHA_MAX, &array16x16[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 3:
                sprintf( str, "PTAT[%2.1f]  32*32 " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array32x32[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_32, TILE_RESO_32);
                update_thermograph(TILE_RESO_32, TILE_RESO_32, TILE_SIZE_HW_32x32, TILE_SIZE_VW_32x32, TILE_ALPHA_MAX, &array32x32[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 4:
                sprintf( str, "PTAT[%2.1f]  64*60 " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array64x60[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_64, TILE_RESO_60);
                update_thermograph(TILE_RESO_64, TILE_RESO_60, TILE_SIZE_HW_64x60, TILE_SIZE_VW_64x60, TILE_ALPHA_MAX, &array64x60[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 5:
                sprintf( str, "PTAT[%2.1f] 160*120" , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array160x120[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_160, TILE_RESO_120);
                update_thermograph(TILE_RESO_160, TILE_RESO_120, TILE_SIZE_HW_160x120, TILE_SIZE_VW_160x120, TILE_ALPHA_MAX, &array160x120[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_DEMO2)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 6:
                sprintf( str, "PTAT[%2.1f] 160*120" , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array160x120[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_160, TILE_RESO_120);
                update_thermograph(TILE_RESO_160, TILE_RESO_120, TILE_SIZE_HW_160x120, TILE_SIZE_VW_160x120, TILE_ALPHA_SWITCH2, &array160x120[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 7:
                sprintf( str, "PTAT[%2.1f] 160*120" , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array160x120[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_160, TILE_RESO_120);
                update_thermograph(TILE_RESO_160, TILE_RESO_120, TILE_SIZE_HW_160x120, TILE_SIZE_VW_160x120, TILE_ALPHA_SWITCH1, &array160x120[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 8:
                sprintf( str, "PTAT[%2.1f] 160*120" , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array160x120[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_160, TILE_RESO_120);
                update_thermograph(TILE_RESO_160, TILE_RESO_120, TILE_SIZE_HW_160x120, TILE_SIZE_VW_160x120, TILE_ALPHA_DEFAULT, &array160x120[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 9:
                sprintf( str, "PTAT[%2.1f]  64*60 " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array64x60[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_64, TILE_RESO_60);
                update_thermograph(TILE_RESO_64, TILE_RESO_60, TILE_SIZE_HW_64x60, TILE_SIZE_VW_64x60, TILE_ALPHA_DEFAULT, &array64x60[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 10:
                sprintf( str, "PTAT[%2.1f]  32*32 " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array32x32[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_32, TILE_RESO_32);
                update_thermograph(TILE_RESO_32, TILE_RESO_32, TILE_SIZE_HW_32x32, TILE_SIZE_VW_32x32, TILE_ALPHA_DEFAULT, &array32x32[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 11:
                sprintf( str, "PTAT[%2.1f]  16*16 " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array16x16[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_16, TILE_RESO_16);
                update_thermograph(TILE_RESO_16, TILE_RESO_16, TILE_SIZE_HW_16x16, TILE_SIZE_VW_16x16, TILE_ALPHA_DEFAULT, &array16x16[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 12:
                sprintf( str, "PTAT[%2.1f]   8*8  " , pdta/10.0 );
                liner_interpolation(&array4x4[0][0], &array8x8[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_8, TILE_RESO_8);
                update_thermograph(TILE_RESO_8, TILE_RESO_8, TILE_SIZE_HW_8x8, TILE_SIZE_VW_8x8, TILE_ALPHA_DEFAULT, &array8x8[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            case 13:
                sprintf( str, "PTAT[%2.1f]   4*4  " , pdta/10.0 );
                update_thermograph(TILE_RESO_4, TILE_RESO_4, TILE_SIZE_HW_4x4, TILE_SIZE_VW_4x4, TILE_ALPHA_DEFAULT, &array4x4[0][0], str);
                sub_phase++;
                if (sub_phase >= SUB_PHASE_DEMO1)
                {
                    sub_phase = 0;
                    phase++;
                }
                break;
            default:
                clear_thermograph("off");
                sub_phase++;
                if (sub_phase >= SUB_PHASE_MAX)
                {
                    sub_phase = 0;
                    phase = 0;
                }
                break;
        }

        ThisThread::sleep_for(PHASE_DELAY);
    }
}

