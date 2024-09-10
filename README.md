# ESP32 WROVER CAM WITH TFT ST7735 1.8" (160 x 128)
In order to use the ST7735 TFT there are several things that need to be adjusted
## WIRING
| TFT ST7735 | ESP32 CAM  |
|------------|------------|
| LED        | 3.3V       |
| SCK        | GPIO 14    |
| SDA        | GPIO 13    |
| AO         | GPIO 32    |
| RESET      | RST        |
| CS         | GPIO 15    |
| GND        | GNC        |
| VCC        | 5V or 3.3V |
## Library Configuration
Add config below
```
// See SetupX_Template.h for all options available
#define USER_SETUP_ID 303

#define ST7735_DRIVER
#define TFT_WIDTH  128
#define TFT_HEIGHT 160
#define ST7735_BLACKTAB
#define TFT_BACKLIGHT_ON HIGH

#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    32  // Data Command control pin
#define TFT_RST   -1  // Reset pin (could connect to RST pin)

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

// Comment out the #define below to stop the SPIFFS filing system and smooth font code being loaded
// this will save ~20kbytes of FLASH
#define SMOOTH_FONT
#define SPI_FREQUENCY  20000000
#define SPI_READ_FREQUENCY  20000000
```
Create a file .h with the above configuration and call it `User_Setup_Select.h`, like following below
```
#include <User_Setups/Setup303_ST7735_ESP32CAM.h>
```