// 4.18.0 0xb491551e
// Generated by imageconverter. Please, do not edit!

#include <BitmapDatabase.hpp>
#include <touchgfx/Bitmap.hpp>

extern const unsigned char image_dark_backgrounds_main_bg_320x240px[]; // BITMAP_DARK_BACKGROUNDS_MAIN_BG_320X240PX_ID = 0, Size: 320x240 pixels

const touchgfx::Bitmap::BitmapData bitmap_database[] = {
    { image_dark_backgrounds_main_bg_320x240px, 0, 320, 240, 0, 0, 320, ((uint8_t)touchgfx::Bitmap::RGB565) >> 3, 240, ((uint8_t)touchgfx::Bitmap::RGB565) & 0x7 }
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
