#include "panel.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "ams568_drv.h"
#include "gpio.h"
#include "tc358779_drv.h"

static panel_stat_t current_panel = {0};
static uint8_t current_timing_index = 0;
static panel_cfg_t current_timing = {0};
static bool current_panel_enabled = 0;
static uint32_t pixel_read_start = 0;
static bool pixel_start_pending = 0;
static uint16_t pixel_start_frame = 0;
static bool pixel_read_pending = 0;
static uint16_t pixel_read_frame = 0;

void panel_reset_state(void) {
   // Set the timing to default until we know better
   current_timing_index = 0;
   panel_get_timing(current_timing_index, &current_timing);

   current_panel.brightness = 255;
   current_panel.mode = LIGHT_MODE_ROLLING_RIGHT_LEFT;
   current_panel.current_limit = AMS568_ACL_OFF;
   current_panel.use_rolling = 1;
   current_panel.reverse_rolling = 1;
   current_panel.high_brightness = 0;
   current_panel.self_refresh = 0;
   current_panel.read_pixel = 0;
   current_panel.direct_pentile = 0;
   // Default to low persistence
   current_panel.persistence = lroundf((float)current_timing.v_total * 0.225f);
   current_panel.lighting_offset = 0;
   current_panel.pixel_settle = 100;  // TODO: determine the actual value
   current_panel.total_rows = current_timing.v_total;
}

void panel_get_state(panel_stat_t* panel) {
   memcpy(panel, &current_panel, sizeof(panel_stat_t));
}

bool panel_set_state(panel_stat_t* panel) {
   memcpy(&current_panel, panel, sizeof(panel_stat_t));
   // Set back the read-only parameters
   current_panel.mode = current_panel.use_rolling ? LIGHT_MODE_ROLLING_RIGHT_LEFT : LIGHT_MODE_GLOBAL;
   current_panel.pixel_settle = 100;  // TODO: determine the actual value
   current_panel.total_rows = current_timing.v_total;
   // Temporarily Set back the parameters that are not yet supported to the default values
   current_panel.reverse_rolling = 1;
   current_panel.lighting_offset = 0;
   // TODO: Allow high brightness mode when #135 is fixed
   current_panel.high_brightness = 0;

   // Only apply the set parameters if the panel is actually on
   if (current_panel_enabled) {
      if (!ams568_configure_brightness(current_panel.use_rolling, current_panel.reverse_rolling,
                                       current_panel.persistence, current_panel.total_rows,
                                       current_panel.brightness, current_panel.high_brightness, current_panel.current_limit)) {
         return 0;
      }
      // The panel may modify the actual number of rows for persistence, so get it back
      current_panel.persistence = ams568_get_persistence();
      current_panel.brightness = ams568_get_brightness();
      if (!ams568_set_video_mode(!current_panel.self_refresh)) {
         return 0;
      }
      if (!ams568_set_direct_pentile(current_panel.direct_pentile)) {
         return 0;
      }
      // TODO: Set global shutter offset, part of #97
   }

   return 1;
}

uint8_t panel_set_brightness(uint8_t brt) {
   current_panel.brightness = brt;
   const uint8_t brt_ctrl[2] = {0x53, 0x20};
   tc358779_mipi_write(MIPI_MODE_GENERIC, brt_ctrl, 2);
   const uint8_t brt_value[2] = {0x51, brt};
   tc358779_mipi_write(MIPI_MODE_GENERIC, brt_value, 2);
   return current_panel.brightness;
}

void panel_power_on(void) {
   LL_GPIO_SetOutputPin(OLED_RST_GPIO_Port, OLED_RST_Pin);

   // LCD needs at least 10ms after unreset
   LL_mDelay(10);
}

void panel_power_off(void) {
   LL_GPIO_ResetOutputPin(OLED_RST_GPIO_Port, OLED_RST_Pin);
}

uint8_t panel_num_timings(void) {
   return ams568_num_supported_timings();
}

void panel_get_timing(uint8_t index, panel_cfg_t* timing) {
   ams568_get_supported_timings(index, timing);
}

uint8_t panel_current_timing(panel_cfg_t* timing) {
   if (timing)
      memcpy(timing, &current_timing, sizeof(panel_cfg_t));

   return current_timing_index;
}

uint8_t panel_get_closest_timing(uint16_t refresh, uint16_t h, uint16_t v) {
   // Unfortunately, the refresh rate is currently the only thing we have
   // available to determine what timing mode we're on, so use that to find
   // the closest timing mode
   uint8_t closest = 0;
   uint16_t diff = UINT16_MAX;
   uint8_t num_timings = panel_num_timings();

   for (uint8_t i = 0; i < num_timings; i++) {
      panel_cfg_t timing;
      panel_get_timing(i, &timing);
      uint16_t new_diff = abs((int16_t)refresh - (int16_t)timing.refresh);
      // Get the closest refresh rate that has the same active size
      // In case of a tie, use the earlier timing since they are basically in
      // order of preference
      if (new_diff < diff && (h == timing.h_active) && (v == timing.v_active)) {
         diff = new_diff;
         closest = i;
      }
   }

   return closest;
}

uint8_t panel_update_timing(uint16_t refresh, uint16_t h, uint16_t v) {
   // Find the supported timing that comes closest to matching what we measured
   current_timing_index = panel_get_closest_timing(refresh, h, v);
   panel_get_timing(current_timing_index, &current_timing);

   // Set the new total rows to be read in the feature report
   current_panel.total_rows = current_timing.v_total;

   // If the refresh rate dropped from the maximum, go into full persistence
   // until set otherwise
   if (current_timing.refresh < ams568_get_default_refresh_rate()) {
      current_panel.persistence = current_panel.total_rows;
   }

   // Change the panel properties that depend on timings
   ams568_configure_brightness(current_panel.use_rolling, current_panel.reverse_rolling,
                               current_panel.persistence, current_panel.total_rows,
                               current_panel.brightness, current_panel.high_brightness, current_panel.current_limit);

   // The panel may modify the actual number of rows for persistence, so get it back
   current_panel.persistence = ams568_get_persistence();
   current_panel.brightness = ams568_get_brightness();

   // Also return the new timing
   return current_timing_index;
}

bool panel_resolution_changed(uint16_t h, uint16_t v) {
   return (h != current_timing.h_active) || (v != current_timing.v_active);
}

void panel_get_size(uint8_t* h, uint8_t* v) {
   *h = PANEL_H;
   *v = PANEL_V;
}

static void panel_pixel_read_reset(void) {
   tc358779_mipi_flush_read();
   pixel_read_start = 0;
   pixel_start_pending = 0;
   pixel_start_frame = 0;
   pixel_read_pending = 0;
   pixel_read_frame = 0;
}

bool panel_enable(bool state) {
   current_panel_enabled = state;

   if (state) {
      // Flush and reset the pixel read queue when starting the panel
      panel_pixel_read_reset();
   }

   // Turn on/off the panel
   if (!s6e3fa0_set_state(state, &current_timing)) {
      // If turning on/off the panel failed, assume the panel is disabled
      current_panel_enabled = 0;
      return 0;
   }

   if (state) {
      // Set the it to use the currently set modes if turning on
      if (!panel_set_state(&current_panel)) {
         return 0;
      }
   }

   return 1;
}

bool is_panel_enabled(void) {
   return current_panel_enabled;
}

// static bool panel_start_read_pixel(vsync_p vsync) {
//    if (current_panel_enabled && current_panel.read_pixel) {
//       // Start a new read when there is a new frame and no read pending
//       if (!pixel_start_pending && (vsync->frame != pixel_start_frame)) {
//          // Defer the triggering the read until 2 ms before the next vsync
//          // to avoid stomping on the value immediately at the start of the
//          // next read, preventing it
//          pixel_read_start = vsync->timestamp + vsync_get_average() - 2000;
//          pixel_start_pending = 1;
//          // We want to match a pixel to a specific timestamp, so mark
//          // which frame number this pixel read is associated with
//          pixel_start_frame = vsync->frame;

//          return 1;
//       }
//    }

//    return 0;
// }

// static bool panel_send_read_pixel(void) {
//    // Read the red value of the top left pixel in the display's framebuffer
//    uint8_t readreg = 0x06;
//    uint8_t result[2] = {0};
//    uint8_t result_len = 1;
//    // Do the read non-blocking, since it can take a whole frame before the
//    // bus turnaround
//    return tc358779_mipi_read(MIPI_MODE_DCS_READ, &readreg, 1, result, &result_len, 0);
// }

// static bool panel_read_pixel(uint8_t* color) {
//    if (pixel_start_pending && timestamper_is_after(pixel_read_start)) {
//       if (panel_send_read_pixel()) {
//          // The pixel read is non-blocking since it has to wait for bus
//          // turnaround on MIPI, so mark the read as pending
//          pixel_read_pending = 1;
//          pixel_read_frame = pixel_start_frame;
//       }
//       pixel_start_pending = 0;
//    }

//    // We read the pixel in a non-blocking way, so check if the read is
//    // ready if there is one pending
//    if (pixel_read_pending) {
//       uint8_t result[2];
//       uint8_t result_len = 1;
//       // This won't succeed until the bus turnaround happens
//       if (tc358779_mipi_complete_read(MIPI_MODE_DCS_READ, 1, result, &result_len)) {
//          *color = result[0];
//          pixel_read_pending = 0;
//          return 1;
//       }
//    }

//    return 0;
// }

// bool panel_get_vsync(vsync_p vsync) {
//    uint8_t ret = 0;
//    vsync_t temp_vsync;

//    // If we're reading the top left pixel value for each frame, we only
//    // return a new frame number and timestamp once we've successfully
//    // read the pixel, which takes around a frame length
//    if (current_panel_enabled && current_panel.read_pixel) {
//       uint8_t color = 0;
//       // We read the pixel in a non-blocking way, so check if the read is
//       // ready if there is one pending
//       if (panel_read_pixel(&color)) {
//          if (vsync_get_frame(vsync, pixel_read_frame)) {
//             vsync->color = color;
//             ret = 1;
//          }
//       }
//    }

//    if (vsync_get(&temp_vsync)) {
//       // Update the refresh rate, since we can't get it any other way
//       vsync_update_average();
//       // Read the pixel from the main loop to avoid i2c/MIPI reads
//       // during the vsync interrupt
//       panel_start_read_pixel(&temp_vsync);

//       // If we're reading pixel values, we only return success when the
//       // top left pixel of this frame has been read
//       if (!g_panel.read_pixel) {
//          // Otherwise, return the vsync we grabbed which contains
//          // just frame number and timestamp
//          ret = 1;
//          memcpy(vsync, &temp_vsync, sizeof(vsync_t));
//       }
//    }

//    return ret;
// }

// uint16_t panel_get_refresh(void) {
//    return vsync_get_refresh();
// }
