// ST7789 using SPI for ESP32-1732S019
#define USER_SETUP_LOADED 1


// Driver
#define ST7789_DRIVER

// Séquence d'initialisation
#define CGRAM_OFFSET

// Ordre des couleurs
#define TFT_RGB_ORDER TFT_BGR

// Inversion des couleurs
#define TFT_INVERSION_ON

// Dimensions de l'écran
#define TFT_WIDTH  170
#define TFT_HEIGHT 320

// ===== CONFIGURATION SPI CRITIQUE =====
// Pins SPI pour ESP32-1732S019 (ESP32-S3)
#define TFT_MOSI 13
#define TFT_SCLK 12
#define TFT_CS   10
#define TFT_DC   11
#define TFT_RST  1
#define TFT_BL   14
#define TFT_MISO -1

// IMPORTANT : Forcer l'utilisation de HSPI sur ESP32-S3
#define USE_HSPI_PORT

// Backlight
#define TFT_BACKLIGHT_ON HIGH

// Fréquences SPI - Commencer LENTEMENT
#define SPI_FREQUENCY       40000000  // 10MHz pour test (au lieu de 40MHz)
#define SPI_READ_FREQUENCY  20000000  // 10MHz
//#define SPI_TOUCH_FREQUENCY 2500000

// Pas de touch screen
#define TOUCH_CS -1

// Chargement des polices
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

// Lissage de police
#define SMOOTH_FONT
