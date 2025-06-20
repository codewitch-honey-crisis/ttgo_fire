#define USE_SPANS
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif
//#define BLUE_FLAME
#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
// this is the handle from the esp panel api
static esp_lcd_panel_handle_t lcd_handle;
#include <gfx.hpp>
#include <uix.hpp>
using namespace gfx;
using namespace uix;
using color_t = gfx::color<typename gfx::rgb_pixel<16>>;
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 135
constexpr static const size_t BUF_WIDTH = (SCREEN_WIDTH / 4);
constexpr static const size_t BUF_HEIGHT = ((SCREEN_HEIGHT / 4) + 6);
#ifdef USE_SPANS
#define PAL_TYPE uint16_t
// store preswapped uint16_ts for performance
#define RGB(r,g,b) (bits::swap(rgb_pixel<16>(r,g,b)))
#else
// store rgb_pixel<16> instances
#define PAL_TYPE rgb_pixel<16>
#define RGB(r,g,b) rgb_pixel<16>(r,g,b)
#endif

// VGA color palette for flames
static PAL_TYPE fire_cols[] = {
    RGB(0, 0, 0), RGB(0, 0, 3), RGB(0, 0, 3), RGB(0, 0, 3),
    RGB(0, 0, 4), RGB(0, 0, 4), RGB(0, 0, 4), RGB(0, 0, 5),
    RGB(1, 0, 5), RGB(2, 0, 4), RGB(3, 0, 4), RGB(4, 0, 4),
    RGB(5, 0, 3), RGB(6, 0, 3), RGB(7, 0, 3), RGB(8, 0, 2),
    RGB(9, 0, 2), RGB(10, 0, 2), RGB(11, 0, 2), RGB(12, 0, 1),
    RGB(13, 0, 1), RGB(14, 0, 1), RGB(15, 0, 0), RGB(16, 0, 0),
    RGB(16, 0, 0), RGB(16, 0, 0), RGB(17, 0, 0), RGB(17, 0, 0),
    RGB(18, 0, 0), RGB(18, 0, 0), RGB(18, 0, 0), RGB(19, 0, 0),
    RGB(19, 0, 0), RGB(20, 0, 0), RGB(20, 0, 0), RGB(20, 0, 0),
    RGB(21, 0, 0), RGB(21, 0, 0), RGB(22, 0, 0), RGB(22, 0, 0),
    RGB(23, 1, 0), RGB(23, 1, 0), RGB(24, 2, 0), RGB(24, 2, 0),
    RGB(25, 3, 0), RGB(25, 3, 0), RGB(26, 4, 0), RGB(26, 4, 0),
    RGB(27, 5, 0), RGB(27, 5, 0), RGB(28, 6, 0), RGB(28, 6, 0),
    RGB(29, 7, 0), RGB(29, 7, 0), RGB(30, 8, 0), RGB(30, 8, 0),
    RGB(31, 9, 0), RGB(31, 9, 0), RGB(31, 10, 0), RGB(31, 10, 0),
    RGB(31, 11, 0), RGB(31, 11, 0), RGB(31, 12, 0), RGB(31, 12, 0),
    RGB(31, 13, 0), RGB(31, 13, 0), RGB(31, 14, 0), RGB(31, 14, 0),
    RGB(31, 15, 0), RGB(31, 15, 0), RGB(31, 16, 0), RGB(31, 16, 0),
    RGB(31, 17, 0), RGB(31, 17, 0), RGB(31, 18, 0), RGB(31, 18, 0),
    RGB(31, 19, 0), RGB(31, 19, 0), RGB(31, 20, 0), RGB(31, 20, 0),
    RGB(31, 21, 0), RGB(31, 21, 0), RGB(31, 22, 0), RGB(31, 22, 0),
    RGB(31, 23, 0), RGB(31, 24, 0), RGB(31, 24, 0), RGB(31, 25, 0),
    RGB(31, 25, 0), RGB(31, 26, 0), RGB(31, 26, 0), RGB(31, 27, 0),
    RGB(31, 27, 0), RGB(31, 28, 0), RGB(31, 28, 0), RGB(31, 29, 0),
    RGB(31, 29, 0), RGB(31, 30, 0), RGB(31, 30, 0), RGB(31, 31, 0),
    RGB(31, 31, 0), RGB(31, 32, 0), RGB(31, 32, 0), RGB(31, 33, 0),
    RGB(31, 33, 0), RGB(31, 34, 0), RGB(31, 34, 0), RGB(31, 35, 0),
    RGB(31, 35, 0), RGB(31, 36, 0), RGB(31, 36, 0), RGB(31, 37, 0),
    RGB(31, 38, 0), RGB(31, 38, 0), RGB(31, 39, 0), RGB(31, 39, 0),
    RGB(31, 40, 0), RGB(31, 40, 0), RGB(31, 41, 0), RGB(31, 41, 0),
    RGB(31, 42, 0), RGB(31, 42, 0), RGB(31, 43, 0), RGB(31, 43, 0),
    RGB(31, 44, 0), RGB(31, 44, 0), RGB(31, 45, 0), RGB(31, 45, 0),
    RGB(31, 46, 0), RGB(31, 46, 0), RGB(31, 47, 0), RGB(31, 47, 0),
    RGB(31, 48, 0), RGB(31, 48, 0), RGB(31, 49, 0), RGB(31, 49, 0),
    RGB(31, 50, 0), RGB(31, 50, 0), RGB(31, 51, 0), RGB(31, 52, 0),
    RGB(31, 52, 0), RGB(31, 52, 0), RGB(31, 52, 0), RGB(31, 52, 0),
    RGB(31, 53, 0), RGB(31, 53, 0), RGB(31, 53, 0), RGB(31, 53, 0),
    RGB(31, 54, 0), RGB(31, 54, 0), RGB(31, 54, 0), RGB(31, 54, 0),
    RGB(31, 54, 0), RGB(31, 55, 0), RGB(31, 55, 0), RGB(31, 55, 0),
    RGB(31, 55, 0), RGB(31, 56, 0), RGB(31, 56, 0), RGB(31, 56, 0),
    RGB(31, 56, 0), RGB(31, 57, 0), RGB(31, 57, 0), RGB(31, 57, 0),
    RGB(31, 57, 0), RGB(31, 57, 0), RGB(31, 58, 0), RGB(31, 58, 0),
    RGB(31, 58, 0), RGB(31, 58, 0), RGB(31, 59, 0), RGB(31, 59, 0),
    RGB(31, 59, 0), RGB(31, 59, 0), RGB(31, 60, 0), RGB(31, 60, 0),
    RGB(31, 60, 0), RGB(31, 60, 0), RGB(31, 60, 0), RGB(31, 61, 0),
    RGB(31, 61, 0), RGB(31, 61, 0), RGB(31, 61, 0), RGB(31, 62, 0),
    RGB(31, 62, 0), RGB(31, 62, 0), RGB(31, 62, 0), RGB(31, 63, 0),
    RGB(31, 63, 0), RGB(31, 63, 1), RGB(31, 63, 1), RGB(31, 63, 2),
    RGB(31, 63, 2), RGB(31, 63, 3), RGB(31, 63, 3), RGB(31, 63, 4),
    RGB(31, 63, 4), RGB(31, 63, 5), RGB(31, 63, 5), RGB(31, 63, 5),
    RGB(31, 63, 6), RGB(31, 63, 6), RGB(31, 63, 7), RGB(31, 63, 7),
    RGB(31, 63, 8), RGB(31, 63, 8), RGB(31, 63, 9), RGB(31, 63, 9),
    RGB(31, 63, 10), RGB(31, 63, 10), RGB(31, 63, 10), RGB(31, 63, 11),
    RGB(31, 63, 11), RGB(31, 63, 12), RGB(31, 63, 12), RGB(31, 63, 13),
    RGB(31, 63, 13), RGB(31, 63, 14), RGB(31, 63, 14), RGB(31, 63, 15),
    RGB(31, 63, 15), RGB(31, 63, 15), RGB(31, 63, 16), RGB(31, 63, 16),
    RGB(31, 63, 17), RGB(31, 63, 17), RGB(31, 63, 18), RGB(31, 63, 18),
    RGB(31, 63, 19), RGB(31, 63, 19), RGB(31, 63, 20), RGB(31, 63, 20),
    RGB(31, 63, 21), RGB(31, 63, 21), RGB(31, 63, 21), RGB(31, 63, 22),
    RGB(31, 63, 22), RGB(31, 63, 23), RGB(31, 63, 23), RGB(31, 63, 24),
    RGB(31, 63, 24), RGB(31, 63, 25), RGB(31, 63, 25), RGB(31, 63, 26),
    RGB(31, 63, 26), RGB(31, 63, 26), RGB(31, 63, 27), RGB(31, 63, 27),
    RGB(31, 63, 28), RGB(31, 63, 28), RGB(31, 63, 29), RGB(31, 63, 29),
    RGB(31, 63, 30), RGB(31, 63, 30), RGB(31, 63, 31), RGB(31, 63, 31)};

// declare the format of the screen
using screen_t = screen<rgb_pixel<16>>;
using color_t = color<typename screen_t::pixel_type>;
// for access to RGB8888 colors which controls use
using color32_t = color<rgba_pixel<32>>;

// UIX allows you to use two buffers for maximum DMA efficiency
// you don't have to, but performance is significantly better
// declare two buffers for transfer
constexpr static const int lcd_buffer_size = bitmap<typename screen_t::pixel_type>::sizeof_buffer({SCREEN_WIDTH,SCREEN_HEIGHT/2});
uint8_t *lcd_transfer_buffer1,*lcd_transfer_buffer2;

// the main screen
screen_t anim_screen;
uint8_t fire_buf[BUF_HEIGHT][BUF_WIDTH]; // VGA buffer, quarter resolution w/extra lines
void fire_on_paint(screen_t::control_surface_type &destination, const srect16 &clip, void* state)
{
 #ifdef USE_SPANS
        static_assert(gfx::helpers::is_same<rgb_pixel<16>,typename screen_t::pixel_type>::value,"USE_SPANS only works with RGB565");
        for (int y = clip.y1; y <= clip.y2; y+=2) {
            // must use rgb_pixel<16>
            // get the spans for the current partial rows (starting at clip.x1)
            // note that we're getting two, because we draw 2x2 squares
            // of all the same color.
            gfx_span row = destination.span(point16(clip.x1,y));
            gfx_span row2 = destination.span(point16(clip.x1,y+1));
            // get the pointers to the partial row data
            uint16_t *prow = (uint16_t*)row.data;
            uint16_t *prow2 = (uint16_t*)row2.data;
            for (int x = clip.x1; x <= clip.x2; x+=2) {
                int i = y >> 2;
                int j = x >> 2;
                PAL_TYPE px = fire_cols[fire_buf[i][j]];
                // set the pixels
                *(prow++)=px;
                // if the clip x ends on an odd value, we need to not set the pointer
                // so check here
                if(x-clip.x1+1<row.length) {
                    *(prow++)=px;
                }
                // the clip y ends on an odd value prow2 will be null
                if(prow2!=nullptr) {
                    *(prow2++)=px;
                    // another check for x if clip ends on an odd value
                    if(x-clip.x1+1<row2.length) {
                        *(prow2++)=px;
                    }
                }                
            }
        }
#else 
        for (int y = clip.y1; y <= clip.y2; ++y) {
            for (int x = clip.x1; x <= clip.x2; ++x) {
                int i = y >> 2;
                int j = x >> 2;
                PAL_TYPE px = fire_cols[fire_buf[i][j]];
                // set the pixel
                destination.point(point16(x,y),px);
            }
        }
#endif               
}

using painter_t = painter<typename screen_t::control_surface_type>;

// the controls
static painter_t fire_painter;

// tell UIX the DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    anim_screen.flush_complete();
    return true;
}
// tell the lcd panel api to transfer data via DMA
static void lcd_on_flush(const rect16 &bounds, const void *bmp, void *state)
{
    int x1 = bounds.x1, y1 = bounds.y1, x2 = bounds.x2 + 1, y2 = bounds.y2 + 1;
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2, y2, (void *)bmp);
}
// initialize the screen using the esp panel API
static void lcd_panel_init()
{
    gpio_set_direction((gpio_num_t)4, GPIO_MODE_OUTPUT);
    //gpio_set_level((gpio_num_t)4,1);
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = 18;
    buscfg.mosi_io_num = 19;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(lcd_buffer_size) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = 16,
    io_config.cs_gpio_num = 5,
    io_config.pclk_hz = 40 * 1000 * 1000,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = 23;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
#else
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
#endif
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_handle);

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)4, 0);
    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, true);
    esp_lcd_panel_set_gap(lcd_handle, 40, 52);
    esp_lcd_panel_mirror(lcd_handle, false, true);
    esp_lcd_panel_invert_color(lcd_handle, true);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
    // Turn on backlight (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)4, 1);
}


// initialize the screens and controls
static void screen_init()
{
    anim_screen.dimensions(ssize16(SCREEN_WIDTH,SCREEN_HEIGHT));
    anim_screen.buffer_size(lcd_buffer_size);
    anim_screen.buffer1(lcd_transfer_buffer1);
    anim_screen.buffer2(lcd_transfer_buffer2);
    const rgba_pixel<32> transparent(0, 0, 0, 0);
    fire_painter.bounds(anim_screen.bounds());
    fire_painter.on_paint_callback(fire_on_paint);
    anim_screen.register_control(fire_painter);
    anim_screen.background_color(color_t::black);
    anim_screen.on_flush_callback(lcd_on_flush);
}
#ifdef ARDUINO
void setup()
{
    Serial.begin(115200);
    Serial.printf("Arduino version: %d.%d.%d\n",ESP_ARDUINO_VERSION_MAJOR,ESP_ARDUINO_VERSION_MINOR,ESP_ARDUINO_VERSION_PATCH);
#else
void loop();
extern "C" void app_main() 
{
    printf("ESP-IDF version %d.%d.%d\n",ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR,ESP_IDF_VERSION_PATCH);
#endif
    lcd_panel_init();

    lcd_transfer_buffer1 = (uint8_t*)heap_caps_malloc(lcd_buffer_size,MALLOC_CAP_DMA);
    lcd_transfer_buffer2 = (uint8_t*)heap_caps_malloc(lcd_buffer_size,MALLOC_CAP_DMA);
    if(lcd_transfer_buffer1==nullptr||lcd_transfer_buffer2==nullptr) {
        printf("Out of memory");
        while(1);
    }
    for (int i = 0; i < BUF_HEIGHT; i++) {
        for (int j = 0; j < BUF_WIDTH; j++) {
            fire_buf[i][j] = 0;
        }
    }
#ifdef BLUE_FLAME
    for(int i = 0;i<256;++i) {
        auto px = fire_cols[i];
        auto tmp = px.template channel<channel_name::R>();
        px.template channel<channel_name::R>(px.template channel<channel_name::B>());
        px.template channel<channel_name::B>(tmp);
        fire_cols[i]=px;
    }
#endif
    // init the UI screen
    screen_init();

#ifndef ARDUINO
    uint32_t ts = pdTICKS_TO_MS(xTaskGetTickCount());
    while(1) {
        if(pdTICKS_TO_MS(xTaskGetTickCount()>=ts+200)) {
            ts = pdTICKS_TO_MS(xTaskGetTickCount());
            vTaskDelay(5);
        }
        loop();
    }
#endif
}

void loop()
{
    static int frames = 0;
    static char szfps[64];
    static uint32_t fps_ts = 0;
    static int old_frames = 0;
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());

    unsigned int i, j, delta;    // looping variables, counters, and data
    static uint32_t total_ms = 0;
    bool pending = anim_screen.flush_pending();
    if(!pending) {
    
        for (i = 1; i < BUF_HEIGHT; ++i)
        {
            for (j = 0; j < BUF_WIDTH; ++j)
            {
                if (j == 0)
                    fire_buf[i - 1][j] = (fire_buf[i][j] +
                                    fire_buf[i - 1][BUF_WIDTH - 1] +
                                    fire_buf[i][j + 1] +
                                    fire_buf[i + 1][j]) >>
                                    2;
                else if (j == SCREEN_WIDTH/4-1)
                    fire_buf[i - 1][j] = (fire_buf[i][j] +
                                    fire_buf[i][j - 1] +
                                    fire_buf[i + 1][0] +
                                    fire_buf[i + 1][j]) >>
                                    2;
                else
                    fire_buf[i - 1][j] = (fire_buf[i][j] +
                                    fire_buf[i][j - 1] +
                                    fire_buf[i][j + 1] +
                                    fire_buf[i + 1][j]) >>
                                    2;

                if (fire_buf[i][j] > 11)
                    fire_buf[i][j] = fire_buf[i][j] - 12;
                else if (fire_buf[i][j] > 3)
                    fire_buf[i][j] = fire_buf[i][j] - 4;
                else
                {
                    if (fire_buf[i][j] > 0)
                        fire_buf[i][j]--;
                    if (fire_buf[i][j] > 0)
                        fire_buf[i][j]--;
                    if (fire_buf[i][j] > 0)
                        fire_buf[i][j]--;
                }
            }
        }
        delta = 0;
        for (j = 0; j < BUF_WIDTH; j++)
        {
            if (rand() % 10 < 5)
            {
                delta = (rand() & 1) * 255;
            }
            fire_buf[BUF_HEIGHT - 2][j] = delta;
            fire_buf[BUF_HEIGHT - 1][j] = delta;
        }
        fire_painter.invalidate();
        ++frames;
    }
    ms = pdTICKS_TO_MS(xTaskGetTickCount());
    anim_screen.update();
    total_ms+=(pdTICKS_TO_MS(xTaskGetTickCount())-ms);

    if (ms > fps_ts + 1000)
    {
        fps_ts = ms;
        if(old_frames!=frames) {
            old_frames = frames;
            if(frames==0) {
                snprintf(szfps, sizeof(szfps), "fps: < 1, total: %d ms",(int)total_ms);
            } else {
                snprintf(szfps, sizeof(szfps), "fps: %d, avg: %d ms", (int)frames,(int)total_ms/frames);
            }
        }
        puts(szfps);
        frames = 0;
        total_ms = 0;
    }

    
}
