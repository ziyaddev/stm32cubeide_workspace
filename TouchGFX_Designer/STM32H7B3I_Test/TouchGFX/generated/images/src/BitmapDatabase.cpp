// 4.18.0 0x50262d6f
// Generated by imageconverter. Please, do not edit!

#include <BitmapDatabase.hpp>
#include <touchgfx/Bitmap.hpp>

extern const unsigned char image_blue_buttons_round_icon_button[]; // BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID = 0, Size: 60x60 pixels
extern const unsigned char image_blue_buttons_round_icon_button_pressed[]; // BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID = 1, Size: 60x60 pixels
extern const unsigned char image_blue_gauges_original_gauge_background_style_00[]; // BITMAP_BLUE_GAUGES_ORIGINAL_GAUGE_BACKGROUND_STYLE_00_ID = 2, Size: 251x251 pixels
extern const unsigned char image_blue_needles_original_gauge_needle_style_00[]; // BITMAP_BLUE_NEEDLES_ORIGINAL_GAUGE_NEEDLE_STYLE_00_ID = 3, Size: 23x66 pixels
extern const unsigned char image_dark_textures_gplaypattern[]; // BITMAP_DARK_TEXTURES_GPLAYPATTERN_ID = 4, Size: 188x178 pixels

const touchgfx::Bitmap::BitmapData bitmap_database[] = {
    { image_blue_buttons_round_icon_button, 0, 60, 60, 13, 11, 34, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 36, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_blue_buttons_round_icon_button_pressed, 0, 60, 60, 13, 11, 34, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 36, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_blue_gauges_original_gauge_background_style_00, 0, 251, 251, 37, 37, 177, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 177, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_blue_needles_original_gauge_needle_style_00, 0, 23, 66, 5, 34, 13, ((uint8_t)touchgfx::Bitmap::ARGB8888) >> 3, 28, ((uint8_t)touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_dark_textures_gplaypattern, 0, 188, 178, 0, 0, 188, ((uint8_t)touchgfx::Bitmap::RGB888) >> 3, 178, ((uint8_t)touchgfx::Bitmap::RGB888) & 0x7 }
};

namespace BitmapDatabase
{
const touchgfx::Bitmap::BitmapData* getInstance()
{
    return bitmap_database;
}

uint16_t getInstanceSize()
{
    return (uint16_t)(sizeof(bitmap_database) / sizeof(touchgfx::Bitmap::BitmapData));
}
} // namespace BitmapDatabase
