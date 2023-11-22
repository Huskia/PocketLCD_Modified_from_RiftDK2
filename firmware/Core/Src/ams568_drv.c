#include "ams568_drv.h"
#include <math.h>
#include <string.h>
#include "main.h"
#include "tc358779_drv.h"

static bool g_initialized = 0;
static uint8_t g_brightness = 0;
static uint16_t g_persistence = 0;

static bool ams568_set_brightness(bool use_hbm, uint8_t brightness);

static inline bool ams568_parameter_pos(uint8_t pos) {
   uint8_t setpos[] = {0xB0, pos};
   return tc358779_mipi_write(MIPI_MODE_GENERIC, setpos, sizeof(setpos));
}

bool ams568_init(const panel_cfg_t* timing) {
   g_initialized = 1;

   // Send the passwords to unlock various modes
   static const uint8_t access_protect_1[] = {0xF0, 0x5A, 0x5A};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, access_protect_1, sizeof(access_protect_1))) {
      return 0;
   }
   static const uint8_t access_protect_2[] = {0xF1, 0x5A, 0x5A};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, access_protect_2, sizeof(access_protect_2))) {
      return 0;
   }
   static const uint8_t access_protect_3[] = {0xFC, 0x5A, 0x5A};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, access_protect_3, sizeof(access_protect_3))) {
      return 0;
   }

   // Stop sleeping
   static const uint8_t sleep_off[] = {0x11};
   if (!tc358779_mipi_write(MIPI_MODE_DCS, sleep_off, sizeof(sleep_off))) {
      return 0;
   }

   // The panel loads data out of NVRAM after sleep out, so give it some time
   LL_mDelay(1);

   // TODO: Only increase the clock if the refresh rate is above ~65 Hz
   if (1) {
      uint8_t delta = 54;
      uint8_t pos = 0x12;
      if (!ams568_parameter_pos(pos)) {
         return 0;
      }
      uint8_t freqread = 0xD7;
      uint8_t freqval[28] = {89};
      uint8_t freqlen = 1;
      if (!tc358779_mipi_read(MIPI_MODE_GENERIC_READ, &freqread, 1, freqval, &freqlen, 1)) {
         return 0;
      }

      uint8_t freqwrite[] = {0xD7, freqval[0] - delta};
      if (!ams568_parameter_pos(pos)) {
         return 0;
      }
      // FIXME: Why does this require multiple writes to succeed?
      for (uint8_t i = 0; i < 10; i++) {
         if (!tc358779_mipi_write(MIPI_MODE_GENERIC, freqwrite, sizeof(freqwrite))) {
            return 0;
         }
      }
      if (!ams568_parameter_pos(0)) {
         return 0;
      }

      // Unknown, sent by Mansu Han
      // Seems to be required for the clock increase to be applied
      static const uint8_t fe1[] = {0xFE, 0x00};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, fe1, sizeof(fe1))) {
         return 0;
      }
      static const uint8_t fe2[] = {0xFE, 0x80};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, fe2, sizeof(fe2))) {
         return 0;
      }
   }

   if (!ams568_set_direct_pentile(0)) {
      return 0;
   }

   // Default to full persistence, minimum brightness
   if (!ams568_configure_brightness(0, 0, timing->v_total - 6, timing->v_total, g_brightness, 0, AMS568_ACL_OFF)) {
      return 0;
   }

   // Enable TE output on vsync
   static const uint8_t te[] = {0x35, 0x00};
   if (!tc358779_mipi_write(MIPI_MODE_DCS, te, sizeof(te))) {
      return 0;
   }

   // Set it to occur on the first vsync row
   uint8_t tescan[] = {0x44, 0x00, 0x01};
   if (!tc358779_mipi_write(MIPI_MODE_DCS, tescan, sizeof(tescan))) {
      return 0;
   }

   // Enable the error pin?
   static const uint8_t err_fg[] = {0xED, 0x0C, 0x04};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, err_fg, sizeof(err_fg))) {
      return 0;
   }

   LL_mDelay(1);

   // Default to MIPI Video mode rather than self refreshing from the frame buffer.
   // This needs to happen 120 ms after sleep out, otherwise the panel
   // power supply isn't ready
   if (!ams568_set_video_mode(1)) {
      return 0;
   }

   return 1;
}

uint8_t ams568_get_default_refresh_rate(void) {
   return 75;
}

uint8_t ams568_num_supported_timings(void) {
   return 4;
}

void ams568_get_supported_timings(uint8_t index, panel_cfg_t* timing) {
   switch (index) {
      case 0:
         // Maximum supported refresh rate for HDCP under 165 MHz
         timing->h_active = 1080;
         // Supporting HDCP requires H blank of 58 pixels or higher
         timing->hfp = 33;
         timing->hsync = 10;
         timing->hbp = 15;
         timing->v_active = 1920;
         timing->vfp = 1;
         timing->vsync = 6;
         timing->vbp = 6;
         timing->refresh = 75;
         break;

      case 1:
         // 60 Hz fallback mode
         timing->h_active = 1080;
         timing->hfp = 33;
         timing->hsync = 10;
         timing->hbp = 15;
         timing->v_active = 1920;
         // Note that the HDMI receiver appears to be much happier when
         // any timings that are switched to live have matching pulse+back
         // lengths
         timing->vfp = 1;
         timing->vsync = 6;
         timing->vbp = 6;
         timing->refresh = 60;
         break;

      case 2:
         // 72 Hz, for a multiple of 24 Hz
         timing->h_active = 1080;
         timing->hfp = 33;
         timing->hsync = 10;
         timing->hbp = 15;
         timing->v_active = 1920;
         timing->vfp = 1;
         timing->vsync = 6;
         timing->vbp = 6;
         timing->refresh = 72;
         break;

      case 3:
         // Half resolution mono mode
         timing->h_active = 1080;
         timing->hfp = 33;
         timing->hsync = 10;
         timing->hbp = 15;
         timing->vfp = 12;
         timing->vsync = 6;
         timing->vbp = 6;
         timing->v_active = (1920 - (timing->vfp + timing->vsync + timing->vbp)) / 2;
         timing->refresh = 120;
         break;
   }

   timing->h_total = timing->h_active + timing->hfp + timing->hsync + timing->hbp;
   timing->v_total = timing->v_active + timing->vfp + timing->vsync + timing->vbp;
}

#define MAX_ELVSS_LIMIT (0.2255f)
#define MIN_ELVSS_LIMIT (0.3f)

static void ams568_limit_brightness(uint8_t* brightness, uint16_t total_rows, uint16_t* persistence, bool* multi_duty) {
   float duty = (float)*persistence / (float)total_rows;
   if (duty < MAX_ELVSS_LIMIT) {
      // No changes necessary if the persistence is short enough to get
      // us below the USB current limit
      return;
   } else if ((duty < MIN_ELVSS_LIMIT)/* || hub_get_self_powered()*/) {
      // Otherwise, we need to reduce brightness by scaling down ELVSS
      if (duty < MIN_ELVSS_LIMIT) {
         int16_t limited = 255 - lroundf((float)*brightness * ((duty - MAX_ELVSS_LIMIT) / (MIN_ELVSS_LIMIT - MAX_ELVSS_LIMIT)));
         if (limited < 0) {
            *brightness = 0;
         } else if (limited > 255) {
            *brightness = 255;
         } else {
            *brightness = limited;
         }
      } else {
         *brightness = 0;
      }
   } else {
      // If we don't have a wall wart and we need to run at full persistence
      // we need to fake high duty cycle by using multiple rolling bands
      *brightness = 0;
      *multi_duty = 1;
      *persistence = lroundf((float)total_rows * MIN_ELVSS_LIMIT);
   }
}

#define MIN_ELVSS        (0x0A)
#define MAX_ELVSS        (0x15)
#define ELVSS_SCALE      (UINT8_MAX / (MAX_ELVSS - MIN_ELVSS))
#define NUM_GAMMA_POINTS (10)

static void gamma_apply_offsets(uint16_t *base_gamma)
{
    // Voltage adjustment at different grey levels
    static const uint8_t gamma_shift[NUM_GAMMA_POINTS] = {
        0,  // 255
        2,  // 203
        4,  // 151
        6,  // 87
        5,  // 51
        4,  // 31
        5,  // 23
        7,  // 11
        12, // 3
        0   // 0
    };
    
    // Color specific shifts at different levels
    static const int8_t color_shift[NUM_GAMMA_POINTS * 3] = {
        -5, 0,  0,  // 255
        1,  1,  0,  // 203
        0,  1,  -1, // 151
        0,  1,  -1, // 87
        -1, 1,  -1, // 51
        0,  2,  -3, // 35
        -1, 2,  -4, // 23
        -2, 6,  -1, // 11
        -2, 12, 8,  // 3
        0,  0,  0   // 0
    };
    
    // Generate the table
    // TODO: Figure out how low persistence effects this
    // TODO: Figure out how to use the OTP shift values
    for (uint8_t i = 0; i < NUM_GAMMA_POINTS; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            base_gamma[i * 3 + j] += gamma_shift[i] + color_shift[i * 3 + j];
        }
    }
}

static void gamma_generate(uint8_t* gamma, uint8_t* mtp) {
   // Starting values for the gamma adjustment table
   uint16_t base_gamma[NUM_GAMMA_POINTS * 3] = {
       0x0100, 0x0100, 0x0100,  // 255
       0x0080, 0x0080, 0x0080,  // 203
       0x0080, 0x0080, 0x0080,  // 151
       0x0080, 0x0080, 0x0080,  // 87
       0x0080, 0x0080, 0x0080,  // 51
       0x0080, 0x0080, 0x0080,  // 31
       0x0080, 0x0080, 0x0080,  // 23
       0x0080, 0x0080, 0x0080,  // 11
       0x0080, 0x0080, 0x0080,  // 3
       0x0000, 0x0000, 0x0000   // 0
   };

   // Apply offsets for the ELVSS and persistence settings
   gamma_apply_offsets(base_gamma);

   // Apply the factory calibrated per-panel gamma offsets
   // gamma_apply_mtp(base_gamma, mtp);

   // Fill in the 16 bit 255 value fields
   for (uint8_t i = 0; i < 3; i++) {
      gamma[i * 2 + 1] = base_gamma[i] >> 8;
      gamma[i * 2 + 2] = base_gamma[i] & 0xFF;
   }

   // Fill in the 8 bit fields for the rest of the table
   for (uint8_t i = 0; i < (NUM_GAMMA_POINTS - 1) * 3; i++) {
      gamma[i + 7] = base_gamma[i + 3] > 255 ? 255 : base_gamma[i + 3];
   }
}

static bool ams568_set_brightness(bool use_hbm, uint8_t brightness) {
   if (use_hbm) {
      // Set to 500 nit brightness
      // Read gamma parameters stored for 255
      uint8_t high_vals[7] = {0};
      uint8_t high_len = 7;
      if (!ams568_parameter_pos(33)) {
         return 0;
      }
      uint8_t hbmread = 0xC8;
      if (!tc358779_mipi_read(MIPI_MODE_GENERIC_READ, &hbmread, 1, high_vals, &high_len, 1)) {
         return 0;
      }

      // Read gamma parameters stored for 203-35
      uint8_t low_vals[15] = {0};
      uint8_t low_len = 15;
      if (!ams568_parameter_pos(72)) {
         return 0;
      }
      hbmread = 0xC8;
      if (!tc358779_mipi_read(MIPI_MODE_GENERIC_READ, &hbmread, 1, low_vals, &low_len, 1)) {
         return 0;
      }
      if (!ams568_parameter_pos(0)) {
         return 0;
      }

      //        // FIXME: Don't use hardcoded values for this
      //        uint8_t high_vals[7] = {0x01, 0x1D, 0x01, 0x18, 0x01, 0x23, 0x05};
      //        uint8_t low_vals[15] = {0x80, 0x80, 0x7F, 0x7C, 0x7C, 0x7C, 0x7A, 0x7B, 0x7B, 0x7E, 0x7E, 0x7E, 0x82, 0x7F, 0x7F};

      // Write the gamma table out
      uint8_t nit[34];
      nit[0] = 0xCA;
      memcpy(nit + 1, high_vals, 6);
      memcpy(nit + 7, low_vals, 15);
      memset(nit + 22, 0x80, 9);
      nit[31] = 0;
      nit[32] = 0;
      nit[33] = 0;
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, nit, sizeof(nit))) {
         return 0;
      }

      // Set the ELVSS HBM parameters from the datasheet
      uint8_t elvss[] = {0xB6, 0x88, 0x0A, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x03, 0x55, 0x54, 0x20,
                         0x00, 0x06, 0x66, 0x6C, 0x0C, high_vals[6]};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, elvss, sizeof(elvss))) {
         return 0;
      }
   } else {
      // The gamma adjustment table setting command
      uint8_t gamma[34] = {0xCA, 0x00};

      // TODO: Read the factory calibrated gamma adjustments from the panel
      uint8_t mtp_gamma[33] = {0};
      //        uint8_t mtp_register = 0xC8;
      //        // Split it into two reads since we have a small MIPI Rx FIFO
      //        uint8_t len = 20;
      //        tc358779_mipi_read(MIPI_MODE_GENERIC_READ, &mtp_register, 1, mtp_gamma, &len, 1);
      //        len = 13;
      //        ams568_parameter_pos(33);
      //        tc358779_mipi_read(MIPI_MODE_GENERIC_READ, &mtp_register, 1, mtp_gamma + 20, &len, 1);
      //        ams568_parameter_pos(0);

      // Generate the gamma table based on brightness and persistence offsets
      gamma_generate(gamma, mtp_gamma);

      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, gamma, sizeof(gamma))) {
         return 0;
      }

      if (!ams568_parameter_pos(0x05)) {
         return 0;
      }
      // Set temperature to 25C
      static const uint8_t temperature[] = {0xB8, 0x19};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, temperature, sizeof(temperature))) {
         return 0;
      }
      if (!ams568_parameter_pos(0)) {
         return 0;
      }

      // Convert a 0 to 255 brightness to an ELVSS voltage setting
      uint8_t elvss = lroundf((float)(UINT8_MAX - brightness) / ELVSS_SCALE) + MIN_ELVSS;
      uint8_t elvss_cmd[] = {0xB6, 0x88, elvss};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, elvss_cmd, sizeof(elvss_cmd))) {
         return 0;
      }
   }

   static const uint8_t update[] = {0xF7, 0x03};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, update, sizeof(update))) {
      return 0;
   }

   return 1;
}

static bool ams568_set_rolling(bool reverse, uint16_t lit_rows, bool multi_duty) {
   // Make the change now if the panel is initialized already
   if (g_initialized) {
      // Setting to switch AID polarity
      // The reverse settings come from Mansu, the forward settings are
      // the default ones read from the device
      if (!ams568_parameter_pos(0x09)) {
         return 0;
      }
      uint8_t aid2[] = {0xCB, reverse ? 0x04 : 0x03};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, aid2, sizeof(aid2))) {
         return 0;
      }
      if (!ams568_parameter_pos(0x23)) {
         return 0;
      }
      uint8_t aid4[] = {0xCB, reverse ? 0x10 : 0x0D};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, aid4, sizeof(aid4))) {
         return 0;
      }
      if (!ams568_parameter_pos(0)) {
         return 0;
      }

      // multi_duty has up to 8 rolling bands going simultaneously, allowing
      // for brightness reduction through duty cycle reduction without
      // introducing flicker
      uint8_t aidctl[] = {0xB2, 0x01, 0x00, lit_rows >> 8, lit_rows & 0xFF, 0x06, 0x06, multi_duty ? 0x78 : 0x08, 0x18, 0x3F, 0xFF, 0xFF};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, aidctl, sizeof(aidctl))) {
         return 0;
      }

      static const uint8_t update[] = {0xF7, 0x03};
      if (!tc358779_mipi_write(MIPI_MODE_GENERIC, update, sizeof(update))) {
         return 0;
      }
   }

   return 1;
}

static bool ams568_set_current_limit(uint8_t mode) {
   // ACL Control - automatic current limiting, which dims the panel
   // if the overall current is too high
   uint8_t acl[] = {0x55, mode};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, acl, sizeof(acl))) {
      return 0;
   }

   return 1;
}

bool ams568_configure_brightness(bool use_rolling, bool reverse_rolling, uint16_t lit_rows, uint16_t total_rows, uint8_t brightness, bool use_hbm, uint8_t current_limit) {
   // Set the correct number of lit rows based on the panel limit and whether we are rolling
   if (use_rolling) {
      if (lit_rows > total_rows - 6)
         lit_rows = total_rows - 6;
   } else {
      lit_rows = total_rows - 6;
      // Default to forward on global shutter
      reverse_rolling = 0;
   }

   bool multi_duty = 0;
   // The panel may reduce the brightness or duty to stay under the current limit
   ams568_limit_brightness(&brightness, total_rows, &lit_rows, &multi_duty);
   bool reducing_brightness = (brightness < g_brightness);

   // If we are reducing brightness, always do that before changing other settings
   if (reducing_brightness) {
      if (!ams568_set_brightness(use_hbm, brightness)) {
         return 0;
      }
   }

   // Set AID (size of the rolling band)
   if (!ams568_set_rolling(reverse_rolling, lit_rows, multi_duty)) {
      return 0;
   }

   if (!ams568_set_current_limit(current_limit)) {
      return 0;
   }

   // Brightness increases come last
   if (!reducing_brightness) {
      if (!ams568_set_brightness(use_hbm, brightness)) {
         return 0;
      }
   }

   g_brightness = brightness;
   // When multi_duty is on, persistence is effectively full
   g_persistence = multi_duty ? total_rows : lit_rows;

   return 1;
}

uint16_t ams568_get_persistence(void) {
   return g_persistence;
}

uint8_t ams568_get_brightness(void) {
   return g_brightness;
}

bool ams568_set_direct_pentile(bool enable) {
   // Enable or disable direct mapping of input video to pentile
   // The mapping when enabled is R1->R1, G1->G1, B1->Null, R2->B2, G2->G2, B2->Null
   uint8_t pentile[] = {0xC0, 0x00, 0x02, 0x03 | (enable << 3), 0x32, 0x03, 0x44, 0x44, 0xC0, 0x00, 0x1C, 0x20, 0xE8};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, pentile, sizeof(pentile))) {
      return 0;
   }

   // Set RE (Readability Enhancement, but it seems to actually be pentile settings
   static const uint8_t re2[] = {0xE3, 0xFF, 0xFF, 0xFF, 0xFF};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, re2, sizeof(re2))) {
      return 0;
   }

   static const uint8_t re3[] = {0xFE, 0x00, 0x03};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, re3, sizeof(re3))) {
      return 0;
   }

   if (!ams568_parameter_pos(0x2B)) {
      return 0;
   }
   static const uint8_t re5[] = {0xFE, 0xE4};
   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, re5, sizeof(re5))) {
      return 0;
   }
   if (!ams568_parameter_pos(0)) {
      return 0;
   }

   return 1;
}

bool ams568_set_video_mode(bool enable) {
   // Completely undocumented command that lets us bypass the frame buffer
   // and directly address the panel in video mode.
   uint8_t display_control[] = {0xF2, enable << 1};  //, timing->vbp, timing->vfp};

   if (!tc358779_mipi_write(MIPI_MODE_GENERIC, display_control, sizeof(display_control))) {
      return 0;
   }

   return 1;
}

bool s6e3fa0_set_state(bool state, const panel_cfg_t* timing) {
   if (state) {
      if (!ams568_init(timing)) {
         return 0;
      }

      // Turn on display
      static const uint8_t display_on[] = {0x29};
      if (!tc358779_mipi_write(MIPI_MODE_DCS, display_on, sizeof(display_on))) {
         return 0;
      }

      return 1;
   } else {
      // Turn off display
      static const uint8_t display_off[] = {0x28};
      if (!tc358779_mipi_write(MIPI_MODE_DCS, display_off, sizeof(display_off))) {
         return 0;
      }

      LL_mDelay(34);

      // Start sleeping.  This is expected to fail if TMDS disappeared,
      // and the panel is getting shut off anyway, so don't
      // bother checking for failure
      static const uint8_t sleep_on[] = {0x10};
      tc358779_mipi_write(MIPI_MODE_DCS, sleep_on, sizeof(sleep_on));
      return 1;
   }
}
