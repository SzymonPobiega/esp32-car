// Small tiled textures for the Fallout theme (RGB565, little endian)
#include "lvgl.h"

// Brushed metal: 7x1 vertical stripes
static const uint8_t fo_metal_map[] = {
  0xC5, 0x49, 0xC5, 0x49, 0xA5, 0x41, 0x84, 0x39, 0x84, 0x39, 0x84, 0x39, 0xA5, 0x41,
};

const lv_img_dsc_t fo_metal_tile = {
  .header = { .cf = LV_IMG_CF_TRUE_COLOR, .always_zero = 0, .reserved = 0, .w = 7, .h = 1 },
  .data_size = sizeof(fo_metal_map),
  .data = fo_metal_map,
};

// CRT scanlines: one dark row every three
static const uint8_t fo_scanline_map[] = {
  0x00, 0x00, 80,
  0x00, 0x00, 0,
  0x00, 0x00, 0,
};

const lv_img_dsc_t fo_scanline_tile = {
  .header = { .cf = LV_IMG_CF_TRUE_COLOR_ALPHA, .always_zero = 0, .reserved = 0, .w = 1, .h = 3 },
  .data_size = sizeof(fo_scanline_map),
  .data = fo_scanline_map,
};
