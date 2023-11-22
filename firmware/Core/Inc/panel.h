#ifndef _PANEL_H_
#define _PANEL_H_

#include <stdint.h>
#include <stdbool.h>
#include "tc358779_drv.h"

typedef struct panel_struct {
    uint16_t persistence;
    uint16_t lighting_offset;
    uint16_t pixel_settle;
    uint16_t total_rows;
    uint8_t brightness;
    uint8_t mode;
    uint8_t current_limit;
    bool use_rolling;
    bool reverse_rolling;
    bool high_brightness;
    bool self_refresh;
    bool read_pixel;
    bool direct_pentile;
} panel_stat_t;

enum {
    LIGHT_MODE_GLOBAL,
    LIGHT_MODE_ROLLING_TOP_BOTTOM,
    LIGHT_MODE_ROLLING_LEFT_RIGHT,
    LIGHT_MODE_ROLLING_RIGHT_LEFT,
    LIGHT_MODE_MAX
};

void panel_reset_state(void);
bool is_panel_enabled(void);
bool panel_resolution_changed(uint16_t h, uint16_t v);
void panel_get_timing(uint8_t index, panel_cfg_t* timing);
uint8_t panel_get_closest_timing(uint16_t refresh, uint16_t h, uint16_t v);
void panel_power_on(void);
void panel_power_off(void);
bool panel_enable(bool state);
uint8_t panel_update_timing(uint16_t refresh, uint16_t h, uint16_t v);
uint8_t panel_current_timing(panel_cfg_t* timing);

void panel_get_size(uint8_t *h, uint8_t *v);
uint8_t panel_num_timings(void);

uint8_t panel_set_brightness(uint8_t brt);

#endif
