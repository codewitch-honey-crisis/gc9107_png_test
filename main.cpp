#include <Arduino.h>
#include <ttgo.hpp>
#include <snake_head.hpp>
using namespace arduino;
using namespace gfx;

// this is basically main() in normal C++
void setup() {
    Serial.begin(115200);
    lcd.initialize();
    lcd.rotation(3);
    
// Create Snakehead sprite
// declare a monochrome bitmap.
constexpr static const size16 snakehead_size(32, 32); 

// declare the sprite:
using sprite_type = sprite<rgb_pixel<16>>;
sprite_type sprite(snakehead_size,snake_head_bw_32x32_data,snake_head_32x32_mask);

// show sprite static on red background
draw::filled_rectangle(lcd, lcd.bounds(), color<rgb_pixel<16>>::red);
draw::sprite(lcd, spoint16(10,50), sprite);
//vTaskDelay(pdMS_TO_TICKS(10000));
}
void loop() {
    // ensure the backlight stays on
    dimmer.wake();
    
}