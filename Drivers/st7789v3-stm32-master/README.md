# ST7789 TFT LCD Driver for STM32

A lightweight, HAL-based driver library for ST7789 TFT LCD displays on STM32 microcontrollers. Supports multiple display sizes, DMA transfers, drawing primitives, text rendering, and image display.

## Supported Displays

| Display Size | Resolution | Define |
|-------------|-----------|--------|
| 0.96 inch | 135x240 | `USING_135X240` |
| 1.3 inch | 240x240 | `USING_240X240` |
| 1.9 inch | 170x320 | `USING_170X320` |
| 2.0 inch | 240x320 | `USING_240X320` |

## Features

- **SPI communication** with optional DMA acceleration
- **4 rotation modes** (0-3)
- **Optional CS pin** control (`CFG_NO_CS`)
- **Drawing primitives** - pixels, lines, rectangles, circles, triangles (outline and filled)
- **Text rendering** with 3 built-in fonts (7x10, 11x18, 16x26)
- **Image display** from flash memory (RGB565)
- **Color inversion** and tear effect control
- **Built-in test function** for quick hardware verification
- **RGB565 color format** with 20+ predefined colors

## Hardware Connection

| ST7789 Pin | STM32 Pin | Description |
|-----------|-----------|-------------|
| SCL/SCK | SPI CLK | SPI Clock |
| SDA/MOSI | SPI MOSI | SPI Data |
| RES | GPIO (configurable) | Reset |
| DC | GPIO (configurable) | Data/Command |
| CS | GPIO (configurable) | Chip Select (optional) |
| BLK | GPIO (configurable) | Backlight (optional) |

## Getting Started

### 1. CubeMX Configuration

- Enable an SPI peripheral in **Transmit Only Master** mode
- If using DMA, add a DMA channel for SPI TX
- Configure GPIO pins for RST, DC, and optionally CS and BLK
- Set pin labels in CubeMX: `ST7789_RST`, `ST7789_DC`, `ST7789_CS`

### 2. Add Files to Project

Copy these files into your project:

```
st7789.c
st7789.h
fonts.c
fonts.h
```

### 3. Configure the Driver

Edit `st7789.h` to match your setup:

```c
/* Select your SPI port */
#define ST7789_SPI_PORT hspi1

/* Enable DMA (comment out to use polling mode) */
#define USE_DMA

/* If CS pin is directly tied to GND, uncomment this */
//#define CFG_NO_CS

/* Select your display size (uncomment ONE) */
//#define USING_135X240
#define USING_240X240
//#define USING_170X320
//#define USING_240X320

/* Select rotation (0-3) */
#define ST7789_ROTATION 2
```

### 4. Initialize and Use

```c
#include "st7789.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();    // if using DMA
    MX_SPI1_Init();

    ST7789_Init();

    // Fill screen with a color
    ST7789_Fill_Color(BLACK);

    // Draw text
    ST7789_WriteString(10, 10, "Hello World!", Font_11x18, WHITE, BLACK);

    // Draw shapes
    ST7789_DrawRectangle(20, 40, 100, 100, CYAN);
    ST7789_DrawFilledCircle(60, 120, 20, RED);

    // Run built-in test
    ST7789_Test();
}
```

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `ST7789_Init()` | Initialize the display |
| `ST7789_SetRotation(uint8_t m)` | Set display rotation (0-3) |

### Drawing

| Function | Description |
|----------|-------------|
| `ST7789_Fill_Color(color)` | Fill entire screen with a color |
| `ST7789_DrawPixel(x, y, color)` | Draw a single pixel |
| `ST7789_DrawPixel_4px(x, y, color)` | Draw a 2x2 pixel block |
| `ST7789_Fill(xSta, ySta, xEnd, yEnd, color)` | Fill a rectangular area |
| `ST7789_DrawLine(x1, y1, x2, y2, color)` | Draw a line |
| `ST7789_DrawRectangle(x1, y1, x2, y2, color)` | Draw a rectangle outline |
| `ST7789_DrawFilledRectangle(x, y, w, h, color)` | Draw a filled rectangle |
| `ST7789_DrawCircle(x0, y0, r, color)` | Draw a circle outline |
| `ST7789_DrawFilledCircle(x0, y0, r, color)` | Draw a filled circle |
| `ST7789_DrawTriangle(x1, y1, x2, y2, x3, y3, color)` | Draw a triangle outline |
| `ST7789_DrawFilledTriangle(x1, y1, x2, y2, x3, y3, color)` | Draw a filled triangle |
| `ST7789_DrawImage(x, y, w, h, data)` | Draw an RGB565 image from memory |

### Text

| Function | Description |
|----------|-------------|
| `ST7789_WriteChar(x, y, ch, font, color, bgcolor)` | Draw a single character |
| `ST7789_WriteString(x, y, str, font, color, bgcolor)` | Draw a string |

Available fonts: `Font_7x10`, `Font_11x18`, `Font_16x26`

### Other

| Function | Description |
|----------|-------------|
| `ST7789_InvertColors(invert)` | Toggle color inversion (0 or 1) |
| `ST7789_TearEffect(tear)` | Enable/disable tearing effect line |
| `ST7789_Test()` | Run built-in display test sequence |

## Predefined Colors (RGB565)

```
WHITE, BLACK, BLUE, RED, MAGENTA, GREEN, CYAN, YELLOW,
GRAY, BRED, GRED, GBLUE, BROWN, BRRED, DARKBLUE,
LIGHTBLUE, GRAYBLUE, LIGHTGREEN, LGRAY, LGRAYBLUE, LBBLUE
```

## DMA Notes

When `USE_DMA` is enabled, the driver uses a small framebuffer for bulk transfers:

```c
#define HOR_LEN  5  // number of horizontal lines buffered
uint16_t disp_buf[ST7789_WIDTH * HOR_LEN];
```

- Increase `HOR_LEN` for faster fills (requires more RAM)
- Set `HOR_LEN` to your full screen height for maximum performance
- Minimum DMA transfer threshold: 16 bytes (smaller writes use polling)

## File Structure

```
st7789.c    - Driver implementation (747 lines)
st7789.h    - Configuration, register definitions, API declarations
fonts.c     - Font bitmap data (7x10, 11x18, 16x26) + sample image
fonts.h     - Font type definitions and declarations
```

## License

This project is open source. Feel free to use it in your projects.
