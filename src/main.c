#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "pngle.h"
#define TEST_IMPLEMENTATION
#include "test.h"
#define TEST2_IMPLEMENTATION
#include "test2.h"

#define LCD_HOST       SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIN_NUM_DATA0          23  //!< for 1-line SPI, this also refereed as MOSI
#define LCD_PIN_NUM_PCLK           18

#define LCD1_PIN_NUM_CS             5
#define LCD1_PIN_NUM_DC             2
#define LCD1_PIN_NUM_RST            15
#define LCD1_PIN_NUM_BK_LIGHT       4

#define LCD2_PIN_NUM_CS             6
#define LCD2_PIN_NUM_DC             3
#define LCD2_PIN_NUM_RST            16
#define LCD2_PIN_NUM_BK_LIGHT       8

#define LCD_X_OFFSET               2
#define LCD_Y_OFFSET               1
#define LCD_INVERT_COLOR
#define LCD_COLOR_SPACE LCD_RGB_ENDIAN_BGR
// The pixel number in horizontal and vertical
#define LCD_H_RES              128
#define LCD_V_RES              115
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

// the following two lines are required for each screen
esp_lcd_panel_handle_t panel_handle1 = NULL;
esp_lcd_panel_io_handle_t io_handle1 = NULL;

esp_lcd_panel_handle_t panel_handle2 = NULL;
esp_lcd_panel_io_handle_t io_handle2 = NULL;

// holds our buffer for sending to the display.
uint8_t fb_data[LCD_V_RES*LCD_H_RES*2];
// the PNG loader
pngle_t* png = NULL;

void init_power() {
#ifdef S3_T_QT
    // for the T-QT Pro. 
    // if the device is battery powered
    // put power init code here
    gpio_config_t pwr_gpio_config;
    memset(&pwr_gpio_config,0,sizeof(pwr_gpio_config));
    pwr_gpio_config.mode = GPIO_MODE_OUTPUT;
    pwr_gpio_config.pin_bit_mask = 1ULL << 4;
    // Initialize the power pin GPIO (T-QT Pro)
    ESP_ERROR_CHECK(gpio_config(&pwr_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)4, 1));
#endif
}

void init_spi() {
    spi_bus_config_t buscfg;
    memset(&buscfg,0,sizeof(buscfg));
    
    buscfg.sclk_io_num = LCD_PIN_NUM_PCLK;
    buscfg.mosi_io_num = LCD_PIN_NUM_DATA0;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_V_RES * LCD_H_RES * 2 + 8;
    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

void init_display(int host, int8_t pin_cs, int8_t pin_dc,int8_t pin_rst,int8_t pin_bkl,esp_lcd_panel_handle_t* out_panel_handle,esp_lcd_panel_io_handle_t* out_io_handle) {
    if(pin_bkl>-1) {
        gpio_config_t bk_gpio_config;
        memset(&bk_gpio_config,0,sizeof(bk_gpio_config));

        bk_gpio_config.mode = GPIO_MODE_OUTPUT;
        bk_gpio_config.pin_bit_mask = 1ULL << pin_bkl;
        // Initialize the GPIO of backlight
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    }
    
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config,0,sizeof(io_config));
    io_config.dc_gpio_num = pin_dc;
    io_config.cs_gpio_num = pin_cs;
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    io_config.lcd_cmd_bits = LCD_CMD_BITS;
    io_config.lcd_param_bits = LCD_PARAM_BITS;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host, &io_config, out_io_handle));

    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config,0,sizeof(panel_config));
    panel_config.reset_gpio_num = pin_rst;
    panel_config.color_space = LCD_COLOR_SPACE;
    panel_config.bits_per_pixel = 16;
    
    // Initialize the LCD configuration
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(*out_io_handle, &panel_config, out_panel_handle));

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)LCD1_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL));

    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(*out_panel_handle));
    esp_lcd_panel_set_gap(*out_panel_handle, LCD_X_OFFSET, LCD_Y_OFFSET);
    // Initialize LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(*out_panel_handle));

#ifdef LCD_SWAP_XY
    // Swap x and y axis (Different LCD screens may need different options)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
#endif

#ifdef LCD_INVERT_COLOR
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(*out_panel_handle, true));
#endif

    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*out_panel_handle, true));
    if(pin_bkl>-1) {
        // Turn on backlight (Different LCD screens may need different levels)
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)pin_bkl, LCD_BK_LIGHT_ON_LEVEL));
    }
    if(out_panel_handle==NULL) {
        printf("Panel not initialized.\n");
        while(1) vTaskDelay(5);
    }

}
void pngle_draw_cb(pngle_t* pngle, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t rgba[4], void* state) {
    int xe = x+w;
    int ye = y+h;
    if(xe>LCD_H_RES) {
        xe = LCD_H_RES;
    }
    if(ye>LCD_V_RES) {
        ye = LCD_V_RES;
    }
    uint8_t a = rgba[3];
    if(a!=255) {
        float af = (float)a/255.0f;
        rgba[0]=roundf(rgba[0]*af);
        rgba[1]=roundf(rgba[1]*af);
        rgba[2]=roundf(rgba[2]*af);
    }
    uint8_t r = rgba[0]>>3;
    uint8_t g = rgba[1]>>2;
    uint8_t b = rgba[1]>>3;
    uint16_t col = (r<<11)|(g<<5)|b;
    // uint16_t col = (b<<11)|(g<<5)|r; // reversed
    uint8_t msb = col>>8;
    uint8_t lsb = col&0xFF;
    size_t stride = LCD_H_RES*2;
    
    uint8_t* start = fb_data+ y*stride;
    for(int py = y;py<ye;++py) {
        uint8_t* x_ptr = start + (x*2);
        for(int px=x;px<xe;++px) {
            *x_ptr++ = msb;
            *x_ptr++ = lsb;
        }
        start+=stride;
    }
}
void init_png() {
    // initialize the png loader
    png = pngle_new();
    if(png==NULL) {
        printf("PNG library not initialized.\n");
        while(1) vTaskDelay(5);
    }
    pngle_set_draw_callback(png,pngle_draw_cb,NULL);
}
// adapt this to read and decode the PNG from the serial stream into fb_data
// void read_png(FILE* handle, uint8_t* data,size_t len) {
//     uint8_t png_feed[1024];
//     size_t png_remain = 0;
//     size_t png_len = 0;
//     while ((png_len = read(handle, png_feed + png_remain, sizeof(png_feed) - png_remain)) > 0) {
//         int fed = pngle_feed(png, png_feed, png_remain + png_len);
//         if (fed < 0) errx(1, "%s", pngle_error(png));

//         png_remain = png_remain + png_len - fed;
//         if (png_remain > 0) memmove(png_feed, png_feed + fed, png_remain);
//     }
// }
void app_main() {
    init_power();   
    init_spi();
    init_display(LCD_HOST, LCD1_PIN_NUM_CS,LCD1_PIN_NUM_DC,LCD1_PIN_NUM_RST,LCD1_PIN_NUM_BK_LIGHT,&panel_handle1,&io_handle1);
    init_display(LCD_HOST, LCD2_PIN_NUM_CS,LCD2_PIN_NUM_DC,LCD2_PIN_NUM_RST,LCD2_PIN_NUM_BK_LIGHT,&panel_handle2,&io_handle2);
    init_png();
    // load the png into fb_data
    pngle_feed(png,test,sizeof(test));
    // call this every time you're done loading a png
    pngle_reset(png);
    // draw it to the first display
    esp_lcd_panel_draw_bitmap(panel_handle1,0,0,LCD_H_RES,LCD_V_RES,fb_data);
    // load the png into fb_data
    pngle_feed(png,test2,sizeof(test2));
    pngle_reset(png);
    // draw it to the second display
    esp_lcd_panel_draw_bitmap(panel_handle2,0,0,LCD_H_RES,LCD_V_RES,fb_data);
    while(1) vTaskDelay(5);
    // not necessary
    pngle_destroy(png);
    
}
