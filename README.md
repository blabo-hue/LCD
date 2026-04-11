# LCDpair
pair an LCD on the base esp32

for Driver ST7796 Setup:

go to .pio -> libdeps -> TFT_eSPI -> User_Setup.h and edit the following:


#define ST7796_DRIVER

// For ESP32 Dev board (only tested with ILI9341 display)
// The hardware SPI can be mapped to any pins

#define TFT_MISO 12


#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15  // Chip select control pin
#define TOUCH_CS 33     // Chip select pin (T_CS) of touch screen
#define TFT_DC    2  // Data Command control pin
//#define TFT_RST   -1  // Reset pin (could connect to RST pin)
#define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST
