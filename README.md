# DDC/CI injector box

This is the code for a box that would let you:

* switch your current monitor input
* change your monitor's brightness
* change your monitor's contrast

with a rotary knob.

Written in Arduino because I value my sanity more than my dignity.

Good reading:

* [VESA DDC/CI](https://vesa.org/vesa-standards/)
* [VESA MCCS](https://vesa.org/vesa-standards/)

## Required physically

* [The DVI I2C checkpoint](https://github.com/parport0/dvi_i2c_checkpoint) or any other way to talk I2C to your monitor. The monitor will usually talk even if there is no computer at the other end of the cable. You can break into VGA, DVI and HDMI cables (or sniff their pins). No luck for DisplayPort which sends I2C over the AUX channel.
* Some RP2040 board. I used the [Adafruit Feather](https://github.com/adafruit/Adafruit-Feather-RP2040-PCB). Maybe RP2350 will work just as well.
* A bi-directional level shifter with pull-ups. I used an identical board to [this](https://cdn.sparkfun.com/datasheets/BreakoutBoards/Logic_Level_Bidirectional.pdf) (at least 2x BSS138 with 10K pull-ups on both high and low voltage sides).
* A display. This code is using Arduino GFX, making it compatible with a lot of displays. I used the [Waveshare 1.47 inch LCD module](https://www.waveshare.com/wiki/1.47inch_LCD_Module) (320x172px panel with ST7789V3).
* A rotary knob with a button (or a rotary knob without a button, and a separate button).

My pinout:

| GPIO pin | Goes to   |
| -------- | --------- |
| 2        | Level-shifted SDA to the monitor |
| 3        | Level-shifted SCL to the monitor |
| 1        | TFT CS    |
| 12       | TFT BL    |
| 0        | TFT RST   |
| 25       | TFT DC    |
| 18       | TFT SCK   |
| 19       | TFT MOSI  |
| 20       | TFT MISO  |
| 8        | Rotary encoder 1 |
| 10       | Rotary encoder 2 |
| 9        | Button    |
| 16       | WS2812    |

## Build

This uses the following libraries:

* [The arduino-pico core of earlephilhower](https://github.com/earlephilhower/arduino-pico)
* [Arduino GFX](https://github.com/moononournation/Arduino_GFX)
* [U8g2](https://github.com/olikraus/u8g2) purely for the font (if you use another font, you don't need this)
* [RotaryEncoder library from mathertel](https://github.com/mathertel/RotaryEncoder) (could be removed IMO)
* [The Adafruit NeoPixel library](https://github.com/adafruit/Adafruit_NeoPixel) because my board has a WS2812 LED (not required at all)

```
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040

arduino-cli lib install "GFX Library for Arduino@1.6.4"
arduino-cli lib install "U8g2@2.35.30"
arduino-cli lib install "RotaryEncoder@1.5.3"
arduino-cli lib install "Adafruit NeoPixel@1.15.1"

arduino-cli compile -e -v --fqbn rp2040:rp2040:adafruit_feather ddcci_injector.ino
```

The build result will be in `build/rp2040.rp2040.adafruit_feather/ddcci_injector.ino.uf2`
