# GR-MANGO_D6T_44L_06_sample
It is a sample program for OMRON MEMS thermal sensors D6T_44L_06 with **2JCIE-EV01** with **GR-MANGO** boards.

D6T_44L_06 is a sensor that can measure the surface temperature without touching the object.

English : https://www.components.omron.com/product-detail?partId=396
日本語 : https://www.omron.co.jp/ecb/product-detail?partNumber=D6T

2JCIE-EV01 sensor evaluation boards are Open Platforms by OMRON corporation and
prototype your ideas with variety environmental information.

English : https://www.components.omron.com/sensor/evaluation-board/2jcie  
日本語 : https://www.omron.co.jp/ecb/sensor/evaluation-board/2jcie  

## Description
Sample programs for acquiring data from sensors on D6T_44L_06.
Output the acquired data to the console and HDMI display.


|Sensor                      |Connect to 2JCIE-EV01|Mode         |Manufacturer      |Interface|
|:---------------------------|:--------------------|:------------|:-----------------|:--------|
|MEMS thermal sensors        |CN5                  |D6T_44L_06   |OMRON             |I2C      |


Example of console display during program execution.  
```
PTAT:   25.3[degC]
25.2, 25.3, 25.2, 24.5,
26.3, 24.5, 23.9, 23.5,
23.6, 24.0, 26.9, 24.9,
25.4, 24.2, 27.4, 25.3,
```

### Terminal setting
|             |         |
|:------------|:--------|
| Baud rate   | 115,200 |
| Data        | 8bit    |
| Parity      | none    |
| Stop        | 1bit    |
| Flow control| none    |


## Development environment (Mbed CLI)
Information of Mbed CLI that includes install&quick start guide is as the following.  
[Installation](https://github.com/ARMmbed/mbed-cli/blob/1.8.3/README.md#installation)  

How to import and build this sample  
```
$ cd <projects directory>
$ mbed import https://github.com/renesas-rz/GR-MANGO_D6T_44L_06_sample
$ cd GR-MANGO_D6T_44L_06_sample
$ mbed compile -m GR_MANGO -t GCC_ARM --profile debug
```

## About custom boot loaders
This sample uses ``custom bootloader`` ``revision 5``, and you can drag & drop the "xxxx_application.bin" file to write the program. Please see [here](https://github.com/d-kato/bootloader_d_n_d) for the detail.  
### How to write program
#### For GR-MANGO
When using ``DAPLink``, please use ``xxxx.bin`` as following.  
1. Connect the ``micro USB type B terminal`` to the PC using a USB cable.
2. You can find the ``MBED`` directory.
3. Drag & drop ``xxxx.bin`` to the ``MBED`` directory.  
4. When writing is completed, press the reset button.  

When using ``custom bootloader``, please use ``xxxx_application.bin`` as following.  
1. Connect the ``USB type C terminal`` to the PC using a USB cable.  
2. Hold down ``USB0`` and press the reset button.  
3. You can find the ``GR-MANG`` directory.  
4. Drag & drop ``xxxx_application.bin`` to the ``GR-MANGO`` directory.  
5. When writing is completed, press the reset button.  

**Attention!**  
For the first time only, you need to write a ``custom bootloader`` using ``DAPLink``.  
