#define USE_SPANS
//#define BLUE_FLAME
#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "gfx.hpp"
#include "uix.hpp"
#include "panel.h"
#include "render_stats.h"
using namespace gfx;
using namespace uix;
using color_t = gfx::color<typename gfx::rgb_pixel<16>>;
constexpr static const size_t BUF_WIDTH = (LCD_WIDTH/ 4);
constexpr static const size_t BUF_HEIGHT = ((LCD_HEIGHT / 4) + 6);
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
void panel_lcd_flush_complete()
{
    anim_screen.flush_complete();
}
static uint32_t stats_interval_buffer[1000];
static uint32_t stats_duration_buffer[1000];
static render_stats_info_t stats;
static void uix_on_flush(const rect16& bounds, const void* bitmap, void* state) {
    panel_lcd_flush(bounds.x1,bounds.y1,bounds.x2,bounds.y2,(void*)bitmap);
}
// initialize the screens and controls
static void screen_init()
{
    anim_screen.dimensions(ssize16(LCD_WIDTH,LCD_HEIGHT));
    anim_screen.buffer_size(LCD_TRANSFER_SIZE);
    anim_screen.buffer1((uint8_t*)panel_lcd_transfer_buffer());
    anim_screen.buffer2((uint8_t*)panel_lcd_transfer_buffer2());
    const rgba_pixel<32> transparent(0, 0, 0, 0);
    fire_painter.bounds(anim_screen.bounds());
    fire_painter.on_paint_callback(fire_on_paint);
    anim_screen.register_control(fire_painter);
    anim_screen.background_color(color_t::black);
    anim_screen.on_flush_callback(uix_on_flush);
}

void loop();
extern "C" void app_main() 
{
    printf("ESP-IDF version %d.%d.%d\n",ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR,ESP_IDF_VERSION_PATCH);

    panel_lcd_init();

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
    // for rendering statistics
    render_stats_init(&stats,stats_interval_buffer,stats_duration_buffer,1000);
    // init the UI screen
    screen_init();

    uint32_t ts = pdTICKS_TO_MS(xTaskGetTickCount());
    while(1) {
        if(pdTICKS_TO_MS(xTaskGetTickCount()>=ts+200)) {
            ts = pdTICKS_TO_MS(xTaskGetTickCount());
            vTaskDelay(5);
        }
        loop();
    }
}

void loop()
{
    
    unsigned int i, j, delta;    // looping variables, counters, and data
    bool pending = anim_screen.flush_pending();
    uint32_t start_ms;
    if(!pending) {
        start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        render_stats_start(&stats,start_ms);
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
                else if (j == BUF_WIDTH-1)
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
    }
    anim_screen.update();
    uint32_t end_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    if(!pending) {
        render_stats_end(&stats,end_ms);
        static uint32_t fps_ts = 0;
        render_stats_t rep;
        if (end_ms > fps_ts + 1000 && render_stats_report(&stats,&rep))
        {
            fps_ts = end_ms;
            float render_time_per_frame = 1000.f/rep.avg_fps;
            float avg_cpu_time = 100.f*(rep.avg_render_ms/(render_time_per_frame-rep.avg_render_ms));
            printf(
                "avg fps: %0.1f\n"
                "1%% lows: %0.1f\n"
                "avg render time: %0.1fms\n"
                "min render time: %0.1fms\n"
                "max render time: %0.1fms\n"
                "avg time per frame: %0.1fms\n"
                "avg cpu time: %0.1f%%\n"
                "\n",
                rep.avg_fps,
                rep.one_pct_low_fps,
                rep.avg_render_ms,
                rep.min_render_ms,
                rep.max_render_ms,
                render_time_per_frame,
                avg_cpu_time
            );
        }
    }
}
