// spans allow for direct bitmap memory
// manipulation for improved efficiency
#define USE_SPANS
// number of frames to keep for perf metrics
#define FPS_FRAMES 100
// use font caching for FPS display
#define FONT_CACHE
// make the flames blue instead of red/orange
//#define BLUE_FLAME

#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_random.h>
#include "gfx.hpp"
#include "uix.hpp"
#include "panel.h"
#include "render_stats.h"
#define TELEGRAMA_IMPLEMENTATION
#include "telegrama.hpp"
#undef TELEGRAMA_IMPLEMENTATION
using namespace gfx;
using namespace uix;
using color_t = gfx::color<typename gfx::rgb_pixel<16>>;
static constexpr const size16 buffer_dim((LCD_WIDTH/ 4), ((LCD_HEIGHT / 4) + 6));
#ifdef USE_SPANS
#define PAL_TYPE uint16_t
// store preswapped uint16_ts for performance
#ifdef BLUE_FLAME
#define RGB(r,g,b) (bits::swap(rgb_pixel<16>(b,g,r)))
#else
#define RGB(r,g,b) (bits::swap(rgb_pixel<16>(r,g,b)))
#endif
#else
// store rgb_pixel<16> instances
#define PAL_TYPE rgb_pixel<16>
#ifdef BLUE_FLAME
#define RGB(r,g,b) rgb_pixel<16>(b,g,r)
#else
#define RGB(r,g,b) rgb_pixel<16>(r,g,b)
#endif
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
static tt_font text_font(telegrama,LCD_HEIGHT/10,font_size_units::px);
#ifdef FONT_CACHE
static font_measure_cache text_measure_cache;
static font_draw_cache text_draw_cache;
#endif
// the main screen
static screen_t anim_screen;
static uint8_t buffer[buffer_dim.height][buffer_dim.width]; // VGA buffer, quarter resolution w/extra lines
static int fps = 0;
static bool show_fps = false;
static uint32_t start_render_ms = 0;
static volatile uint32_t end_render_ms = 0;
static uint32_t cpu_time_accum = 0;
static int cpu_time_count = 0;
// set true once we've called render_stats_start for a frame
// and are waiting for the final flush to complete
static volatile bool render_pending = false;
// set true on the iteration where we rendered + called update(),
// cleared after we capture the CPU end timestamp
static bool cpu_stamp_needed = false;
static void fire_on_paint(screen_t::control_surface_type &destination, const srect16 &clip, void* state)
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
                PAL_TYPE px = fire_cols[buffer[i][j]];
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
                PAL_TYPE px = fire_cols[buffer[i][j]];
                // set the pixel
                destination.point(point16(x,y),px);
            }
        }
#endif   
    if(show_fps) {
        static const int16_t y1 = LCD_HEIGHT-(LCD_HEIGHT/10) - 2;
        static const srect16 fps_rect(2,y1,LCD_WIDTH-3,y1+(LCD_HEIGHT/10));
        if(clip.intersects(fps_rect)) {
            static char fps_text[64];
            snprintf(fps_text,sizeof(fps_text),"fps: %d",fps);
#ifdef FONT_CACHE
            text_info ti(fps_text,text_font,4,text_encoding::utf8,&text_measure_cache,&text_draw_cache);
#else
            text_info ti(fps_text,text_font);
#endif
#ifdef BLUE_FLAME
            static const auto fps_col = color_t::red;
#else
           static const auto fps_col = color_t::blue;
#endif
            draw::text(destination,fps_rect,ti,fps_col);
        }
    }
}

using painter_t = painter<typename screen_t::control_surface_type>;

// the controls
static painter_t fire_painter;

// tell UIX the DMA transfer is complete
void panel_lcd_flush_complete()
{
    anim_screen.flush_complete();
    // always overwrite - if there are multiple flushes per frame,
    // we want the timestamp of the LAST one
    end_render_ms = pdTICKS_TO_MS(xTaskGetTickCountFromISR());
}
static uint32_t stats_interval_buffer[FPS_FRAMES];
static uint32_t stats_duration_buffer[FPS_FRAMES];
static render_stats_info_t stats;
static void uix_on_flush(const rect16& bounds, const void* bitmap, void* state) {
    panel_lcd_flush(bounds.x1,bounds.y1,bounds.x2,bounds.y2,(void*)bitmap);
}
#if defined(TOUCH_BUS) || defined(BUTTON)
#define HAS_INPUT
static TickType_t pressed = 0;
static void update_input() {
#ifdef TOUCH_BUS
    panel_touch_update();
    uint16_t x,y,s;
    size_t count = 1;
    panel_touch_read_raw(&count,&x,&y,&s);
    if(count>0) {
        if(pressed==0) {
            pressed = xTaskGetTickCount();
        }
    } else if(pressed>0) {
        show_fps = !show_fps;
        pressed = 0;
    }
#endif
#ifdef BUTTON
    if(panel_button_read_all()) {
        if(pressed==0) {
            pressed = xTaskGetTickCount();
        }
    } else {
        if(pressed>0) {
            show_fps = !show_fps;
            pressed = 0;
        }
    }
#endif
}
#endif
// initialize the screens and controls
static void screen_init()
{
    anim_screen.dimensions(ssize16(LCD_WIDTH,LCD_HEIGHT));
    anim_screen.buffer_size(LCD_TRANSFER_SIZE);
    anim_screen.buffer1((uint8_t*)panel_lcd_transfer_buffer());
    anim_screen.buffer2((uint8_t*)panel_lcd_transfer_buffer2());
    fire_painter.bounds(anim_screen.bounds());
    fire_painter.on_paint_callback(fire_on_paint);
    anim_screen.register_control(fire_painter);
    anim_screen.background_color(color_t::black);
    anim_screen.on_flush_callback(uix_on_flush);
}

void loop();
void loop_task(void* arg) {
    TickType_t ts = xTaskGetTickCount();
    static const TickType_t delay = pdMS_TO_TICKS(200);
    while(1) {
        // ping the wdt so it doesn't get lonely
        TickType_t new_ts = xTaskGetTickCount();
        if(new_ts >= ts + delay) {
            ts = new_ts;
            vTaskDelay(5);
        }
        loop();
    }
}
extern "C" void app_main() 
{
    printf("ESP-IDF version %d.%d.%d\n",ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR,ESP_IDF_VERSION_PATCH);

#ifdef BUTTON
    panel_button_init();
#endif
#ifdef POWER
    panel_power_init();
#endif
#ifdef EXPANDER_BUS
    panel_expander_init();
#endif
    panel_lcd_init();
#ifdef TOUCH_BUS
    panel_touch_init();
#endif
#ifdef LCD_BCKL_PWM
    panel_lcd_backlight(64);
#endif

    if(gfx_result::success!=text_font.initialize()) {
        puts("OUT OF MEMORY");
        esp_restart();
    }
#ifdef FONT_CACHE
    text_measure_cache.max_entries(20);
    text_measure_cache.initialize();
    text_draw_cache.max_entries(20);
    text_draw_cache.initialize();
#endif
    
    // for rendering statistics
    render_stats_init(&stats,stats_interval_buffer,stats_duration_buffer,FPS_FRAMES);
    // init the UI screen
    screen_init();

    TaskHandle_t handle;
    xTaskCreate(loop_task,"loop_task",8192,nullptr,uxTaskPriorityGet(NULL),&handle);
}

void loop()
{
    
    unsigned int i, j, delta;    // looping variables, counters, and data
    bool flush_pending = anim_screen.flush_pending();
    if(!flush_pending) {
        // if we were waiting for DMA to finish, grab the end timestamp
        // BEFORE starting the next frame's timing. This ensures the
        // start/end pair fed to render_stats belongs to the same frame.
        if(render_pending) {
            // end_render_ms was set by the ISR (possibly multiple times
            // if UIX split the frame into multiple DMA chunks - we want
            // the last one, which is what's sitting in end_render_ms now)
            uint32_t ems = end_render_ms;
            if(ems != 0) {
                render_stats_end(&stats, ems);
                end_render_ms = 0;
                render_pending = false;
                static uint32_t fps_ts = 0;
                render_stats_t rep;
                if (ems > fps_ts + 1000 && render_stats_report(&stats,&rep))
                {
                    fps_ts = ems;
                    float render_time_per_frame = 1000.f/rep.avg_fps;
                    float avg_cpu_time = 0;
                    float cpu_pct = 0;
                    if(cpu_time_count > 0) {
                        avg_cpu_time = (float)cpu_time_accum / cpu_time_count;
                        cpu_pct = 100.f * (avg_cpu_time / render_time_per_frame);
                    }
                    cpu_time_accum = 0;
                    cpu_time_count = 0;
                    printf(
                        "avg fps: %0.1f\n"
                        "1%% lows: %0.1f\n"
                        "avg render time: %0.1fms\n"
                        "min render time: %0.1fms\n"
                        "max render time: %0.1fms\n"
                        "avg cpu time: %0.1fms %0.1f%%\n"
                        "avg time per frame: %0.1fms\n"
                        "\n",
                        rep.avg_fps,
                        rep.one_pct_low_fps,
                        rep.avg_render_ms,
                        rep.min_render_ms,
                        rep.max_render_ms,
                        avg_cpu_time,
                        cpu_pct,
                        render_time_per_frame
                    );
                    fps = roundf(rep.avg_fps);
                }
            }
        }
        // now start timing the new frame
        start_render_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        render_stats_start(&stats, start_render_ms);
        render_pending = true;
        for (i = 1; i < buffer_dim.height; ++i)
        {
            for (j = 0; j < buffer_dim.width; ++j)
            {
                if (j == 0)
                    buffer[i - 1][j] = (buffer[i][j] +
                                    buffer[i - 1][buffer_dim.width - 1] +
                                    buffer[i][j + 1] +
                                    buffer[i + 1][j]) >>
                                    2;
                else if (j == buffer_dim.width-1)
                    buffer[i - 1][j] = (buffer[i][j] +
                                    buffer[i][j - 1] +
                                    buffer[i + 1][0] +
                                    buffer[i + 1][j]) >>
                                    2;
                else
                    buffer[i - 1][j] = (buffer[i][j] +
                                    buffer[i][j - 1] +
                                    buffer[i][j + 1] +
                                    buffer[i + 1][j]) >>
                                    2;

                if (buffer[i][j] > 11)
                    buffer[i][j] = buffer[i][j] - 12;
                else if (buffer[i][j] > 3)
                    buffer[i][j] = buffer[i][j] - 4;
                else
                {
                    if (buffer[i][j] > 0)
                        buffer[i][j]--;
                    if (buffer[i][j] > 0)
                        buffer[i][j]--;
                    if (buffer[i][j] > 0)
                        buffer[i][j]--;
                }
            }
        }
        delta = 0;
        for (j = 0; j < buffer_dim.width; j++)
        {
            if (esp_random() % 10 < 5)
            {
                delta = (esp_random() & 1) * 255;
            }
            buffer[buffer_dim.height - 2][j] = delta;
            buffer[buffer_dim.height - 1][j] = delta;
        }
        fire_painter.invalidate();
        // flag that we need to capture CPU time after update()
        cpu_stamp_needed = true;
    }
    anim_screen.update();
    // capture CPU end time exactly once - right after the update() call
    // on the iteration where we actually did the rendering work.
    // This captures fire sim + paint callbacks + flush initiation,
    // but NOT the time spent waiting for DMA to complete.
    if(cpu_stamp_needed) {
        uint32_t cpu_end = pdTICKS_TO_MS(xTaskGetTickCount());
        cpu_time_accum += (cpu_end - start_render_ms);
        cpu_time_count++;
        cpu_stamp_needed = false;
    }
#ifdef HAS_INPUT
    update_input();
#endif
}