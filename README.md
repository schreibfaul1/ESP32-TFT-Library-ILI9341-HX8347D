# ESP32-TFT-Library-ILI9431-HX8347D
TFT Library for controller ILI9341 and HX8347D

Displays working with this library:
![Display](https://github.com/schreibfaul1/ESP32-TFT-Library-ILI9431-HX8347D/blob/master/additional%20info/tested%20displays.jpg)

Create new fonts with MikroElektronika GLCD Font Creator and insert the new font in fonts.h
You can also display bitmaps, touchpadcontroller XPT2046 is included

Use the touchpad, if the display have one
```` c++
#include "Arduino.h"
#include "SPI.h"
#include "tft.h"

#define TP_IRQ        39
#define TP_CS         16

TFT tft(1); // Waveshare 2.8 TFT with TP
TP tp(TP_CS, TP_IRQ);

uint16_t tp_x, tp_y;

void setup() {
    SPI.begin();
    tft.begin();
    tft.setRotation(3); // Use landscape format
    tp.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREENYELLOW);
    tft.setTextSize(2);
}

//-------------------------------------------------------------------------------------
void loop(void) {
    tp.loop();
}
//-------------------------------------------------------------------------------------

// Event from TouchPad
void tp_pressed(uint16_t x, uint16_t y){
    tp_x=x;  tp_y=y;
}
void tp_released(){
    tft.fillRect(100, 100, 80, 40, TFT_BLACK);
    tft.setCursor(100, 100);
    tft.print("PosX="); tft.println(tp_x);
    tft.print("PosY="); tft.println(tp_y);
}
````
