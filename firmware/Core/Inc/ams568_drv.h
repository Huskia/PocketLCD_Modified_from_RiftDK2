#ifndef _AMS568_DRV_H_
#define _AMS568_DRV_H_

#include <stdint.h>
#include <stdbool.h>
#include "panel.h"
#include "tc358779_drv.h"

// Support for the Samsung AMS568 5.7" AMOLED
#define PANEL_H           71
#define PANEL_V           126

enum {
    AMS568_ACL_OFF = 0,
    AMS568_ACL_30 = 1,
    AMS568_ACL_25 = 2,
    AMS568_ACL_50 = 3
};

bool ams568_init(const panel_cfg_t* timing);
uint8_t ams568_get_default_refresh_rate(void);
uint8_t ams568_num_supported_timings(void);
void ams568_get_supported_timings(uint8_t index, panel_cfg_t* timing);
bool ams568_configure_brightness(bool use_rolling, bool reverse_rolling, uint16_t lit_rows, uint16_t total_rows, uint8_t brightness, bool use_hbm, uint8_t current_limit);
uint16_t ams568_get_persistence(void);
uint8_t ams568_get_brightness(void);
bool ams568_set_direct_pentile(bool enable);
bool ams568_set_video_mode(bool enable);
bool s6e3fa0_set_state(bool state, const panel_cfg_t* timing);

#endif
