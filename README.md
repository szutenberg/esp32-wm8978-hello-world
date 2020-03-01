# ESP32 & WM8978 Hello World 

Simple "hello world" project which allows you to get started with WM8978 and ESP32. 

In this example, 100ms beep are generated on the left channel (LOUT1) every second.

## Hardware
Example works out of-the-box with TTGO-TAudio T9 board. Connect speaker or headphones to mini-jack socket. 

## Software 
This example works with esp-idf v.4.0.

## Expected Output

```
I (321) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (332) gpio: GPIO[0]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (342) WM8978: Init
I (342) WM8978: I2C status 0
I (352) WM8978: Soft reset successfull
I (362) WM8978: Init finished
I (372) I2S: DMA Malloc info, datalen=blocksize=2048, dma_buf_count=3
I (372) I2S: PLL_D2: Req RATE: 48000, real rate: 48076.000, BITS: 16, CLKM: 13, BCK: 8, MCLK: 12292917.167, SCLK: 1538432.000000, diva: 64, divb: 1
I (5382) app: Heart beat
I (10382) app: Heart beat
```

## Authors
* Michal Szutenberg ( http://github.com/szutenberg )
* unknown Chinese author of WM8978 functions (code originally written for Arduino can be found on many websites)