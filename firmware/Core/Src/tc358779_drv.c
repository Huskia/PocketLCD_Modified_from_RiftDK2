#include "tc358779_drv.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "edid.h"
#include "gpio.h"
#include "i2c.h"
#include "panel.h"

#define TC_SLA 0x0F

#define UUID_ADDR_0 ((uint32_t *)0x1FF80050)
#define UUID_ADDR_1 ((uint32_t *)0x1FF80054)
#define UUID_ADDR_2 ((uint32_t *)0x1FF80064)

uint8_t v_lock_changed = 0;
uint8_t brightness_set = 255;

typedef struct tc358779_struct {
   uint8_t rx_buf[64];
   uint8_t tx_buf[64];
   uint8_t command_buf[64];
   uint8_t command_len;
   uint8_t command_type;
   uint8_t mipi_mode;
   uint8_t max_return;
} tc358779_t;

tc358779_t g_h2c = {{0}};

#define MAX_CACHED_COMMAND 64
#define USE_MANUAL_HPDO    0
#define USE_PULSE          1
#define USE_VIP            0

// PLL Clock Speed
// float(refclk)*((fbd+1.0)/(prd+1.0))*(1.0/(2**frs))
#if USE_VIP
// 913 MHz
#define FBD 168
#define PRD 4
#else /* USE_VIP */
// 1.053 GHz
#define FBD 194
#define PRD 4
#endif /* USE_VIP */
// static const float dsiclk = 27.0F * (float)(FBD + 1) / (float)(PRD + 1) / 2.0F;

static void tc358779_wr_reg(uint16_t reg, const uint8_t* data, uint8_t len) {
   uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
   i2c_master_tx(I2C2, TC_SLA, reg_buf, 2, (uint8_t*)data, len);
}

static void tc358779_wr_reg_byte(uint16_t reg, const uint8_t data) {
   uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
   i2c_master_tx(I2C2, TC_SLA, reg_buf, 2, (uint8_t*)&data, 1);
}

static void tc358779_rd_reg(uint16_t reg, uint8_t* data, uint8_t len) {
   uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
   i2c_master_rx(I2C2, TC_SLA, reg_buf, 2, data, len);
}

// Configuration writes for DSI require writing to a different register than
// we read from.  These write appear to be failing occasionally, so we need
// to verify that they succeed
static bool tc358779_confw_verify(const uint8_t* command) {
   // Find the target register
   uint16_t addr = 0;
   switch (command[3] & 0x1F) {
      case 0x03:
         addr = TC_CSI_DSI_CONTROL;
         break;
      case 0x12:
         addr = TC_DSI_RXERR_HALT;
         break;
      case 0x15:
         addr = TC_CSI_DSI_ERR_HALT;
         break;
      default:
         return 0;
   }

   // Check if this is a set or clear on the bits
   bool setting = (command[3] & 0xA0) == 0xA0;

   // Read the target register
   uint8_t confw_ver[2] = {0x00};
   tc358779_rd_reg(addr, confw_ver, sizeof(confw_ver));

   // Check if the bits were set or cleared successfully
   for (uint8_t i = 0; i < 3; i++) {
      if (setting) {
         if ((command[i] & confw_ver[i]) != command[i])
            return 0;
      } else {
         if (command[i] & confw_ver[i])
            return 0;
      }
   }

   return 1;
}

#if USE_VIP
static void tc358779_vip_init(const panel_timing_p input, const panel_timing_p panel) {
   // Enable VIP for upscaling and clear flags
   static const uint8_t vip_control[] = {0x78, 0x00, 0x0F, 0x00};
   tc358779_wr_reg(TC_VIP_CONTROL, vip_control, sizeof(vip_control));

   // Bypass the deinterlacer, color converter
   static const uint8_t bypass[] = {0x00, 0x01, 0x10, 0x00};
   tc358779_wr_reg(TC_VBEMS_COM_TEST, bypass, sizeof(bypass));

   // Set up the LCD Controller part of the VIP with the desired output timings
   // Set line timing from VBP + VSW - 2
   uint8_t go_lines[] = {panel->vbp + panel->vsync - 2, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_GO_LINES, go_lines, sizeof(go_lines));
   // From the spreadsheet, clock cycles to delay VSYNC
   static const uint8_t vd_delay[] = {0x3C, 0x2E, 0x00, 0x00};
   tc358779_wr_reg(TC_VD_DELAY, vd_delay, sizeof(vd_delay));
   // Set LCD controller parameters
   uint8_t vsw[] = {panel->vsync - 1, ((panel->vsync - 1) >> 8) & 0x07, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_VSW, vsw, sizeof(vsw));
   uint8_t vbp[] = {panel->vbp - 1, ((panel->vbp - 1) >> 8) & 0x07, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_VBP, vbp, sizeof(vbp));
   uint8_t val[] = {panel->v_active - 1, ((panel->v_active - 1) >> 8) & 0x0F, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_VAL, val, sizeof(val));
   uint8_t vfp[] = {panel->v_front - 1, ((panel->v_front - 1) >> 8) & 0x07, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_VFP, vfp, sizeof(vfp));

   // Formulas from the spreadsheet
   float pclk = panel->h_total * panel->v_total * panel->refresh / 1000000.0F;
   uint16_t hsw = lroundf((float)panel->hsync * dsiclk / pclk) - 1;
   uint16_t hbp = lroundf((float)panel->hbp * dsiclk / pclk) - 1;
   uint16_t hfp = lroundf((float)panel->h_front * dsiclk / pclk + (float)panel->h_active * dsiclk / pclk) - panel->h_active - 1;
   uint8_t viphsw[] = {hsw, (hsw >> 8) & 0x0F, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_HSW, viphsw, sizeof(viphsw));
   uint8_t viphbp[] = {hbp, (hbp >> 8) & 0x0F, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_HBP, viphbp, sizeof(viphbp));
   uint8_t hap[] = {panel->h_active - 1, ((panel->h_active - 1) >> 8) & 0x0F, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_HAP, hap, sizeof(hap));
   uint8_t viphfp[] = {hfp, (hfp >> 8) & 0x0F, 0x00, 0x00};
   tc358779_wr_reg(TC_VIP_HFP, viphfp, sizeof(viphfp));

   // Set up the coefficients for the scaler
   uint8_t yhvsin[] = {input->v_active, (input->v_active >> 8) & 0x3F, input->h_active, (input->h_active >> 8) & 0x1F};
   tc358779_wr_reg(TC_CS_YHVSIN, yhvsin, sizeof(yhvsin));
   tc358779_wr_reg(TC_CS_CHVSIN, yhvsin, sizeof(yhvsin));
   uint8_t hszout[] = {panel->v_active, (panel->v_active >> 8) & 0x0F, panel->h_active, (panel->h_active >> 8) & 0x07};
   tc358779_wr_reg(TC_CS_HSZOUT, hszout, sizeof(hszout));

   uint8_t yhfilmode[] = {0x01, 0x00, panel->h_active, (panel->h_active >> 8) & 0x07};
   tc358779_wr_reg(TC_CS_YHFILMODE, yhfilmode, sizeof(yhfilmode));
   uint8_t yhfilpsmode[] = {input->h_active, (input->h_active >> 8) & 0x1F, 0x00, 0x00};
   tc358779_wr_reg(TC_CS_YHFILPSMODE, yhfilpsmode, sizeof(yhfilpsmode));
   // I have no idea what this is, but we need to zero the three LSB
   uint32_t filbase = (((input->h_active - 1) << 16) / (panel->h_active - 1)) & 0x003FFFF8;
   uint8_t yhfilbase[] = {filbase, filbase >> 8, filbase >> 16, 0x00};
   tc358779_wr_reg(TC_CS_YMHFILBASE, yhfilbase, sizeof(yhfilbase));
   // The C registers get the same settings as the Y ones
   tc358779_wr_reg(TC_CS_CHFILMODE, yhfilmode, sizeof(yhfilmode));
   tc358779_wr_reg(TC_CS_CHFILPSMODE, yhfilpsmode, sizeof(yhfilpsmode));
   tc358779_wr_reg(TC_CS_CMHFILBASE, yhfilbase, sizeof(yhfilbase));

   uint8_t yvfilmode[] = {0x01, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_CS_YVFILMODE, yvfilmode, sizeof(yvfilmode));
   uint8_t yvfilpsmode[] = {input->v_active, (input->v_active >> 8) & 0x3F, 0x00, 0x00};
   tc358779_wr_reg(TC_CS_YVFILPSMODE, yvfilpsmode, sizeof(yvfilpsmode));
   // I have no idea what this is, but we need to zero the three LSB
   uint32_t vfilbase = (((input->v_active - 1) << 16) / (panel->v_active - 1)) & 0x003FFFF8;
   uint8_t yvfilbase[] = {vfilbase, vfilbase >> 8, vfilbase >> 16, 0x00};
   tc358779_wr_reg(TC_CS_YVFILBASE, yvfilbase, sizeof(yvfilbase));
   // The C registers get the same settings as the Y ones
   tc358779_wr_reg(TC_CS_CVFILMODE, yvfilmode, sizeof(yvfilmode));
   tc358779_wr_reg(TC_CS_CVFILPSMODE, yvfilpsmode, sizeof(yvfilpsmode));
   tc358779_wr_reg(TC_CS_CVFILBASE, yvfilbase, sizeof(yvfilbase));
}
#endif /* USE_VIP */

void tc358779_power_on(void) {
   LL_GPIO_SetOutputPin(TC_PWR_GPIO_Port, TC_PWR_Pin);
   LL_GPIO_SetOutputPin(TC_STDBY_GPIO_Port, TC_STDBY_Pin);
   LL_mDelay(50);
   // Wait 50 ms after VDDC, VDDIO1, and VDDIO2 are there before coming
   // out of reset
   LL_GPIO_SetOutputPin(TC_RST_GPIO_Port, TC_RST_Pin);
   LL_mDelay(10);
}

static void tc358779_mipi_config(void) {
#if USE_VIP
   // Setup specific MIPI protocol timings
   static const uint8_t lineinitcnt[] = {0x00, 0x2D, 0x00, 0x00};
   tc358779_wr_reg(TC_LINEINITCNT, lineinitcnt, sizeof(lineinitcnt));
   static const uint8_t lptxtimecnt[] = {0x05, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_LPTXTIMECNT, lptxtimecnt, sizeof(lptxtimecnt));
   static const uint8_t tclkheadercnt[] = {0x04, 0x20, 0x00, 0x00};
   tc358779_wr_reg(TC_TCLK_HEADERCNT, tclkheadercnt, sizeof(tclkheadercnt));
   static const uint8_t tclktrailcnt[] = {0x03, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_TCLK_TRAILCNT, tclktrailcnt, sizeof(tclktrailcnt));
   static const uint8_t thsheadercnt[] = {0x06, 0x06, 0x00, 0x00};
   tc358779_wr_reg(TC_THS_HEADERCNT, thsheadercnt, sizeof(thsheadercnt));
   static const uint8_t twakeup[] = {0x00, 0x4A, 0x00, 0x00};
   tc358779_wr_reg(TC_TWAKEUP, twakeup, sizeof(twakeup));
#else  /* USE_VIP */
   // Setup specific MIPI protocol timings
   static const uint8_t lineinitcnt[] = {0x00, 0x2C, 0x00, 0x00};
   tc358779_wr_reg(TC_LINEINITCNT, lineinitcnt, sizeof(lineinitcnt));
   static const uint8_t lptxtimecnt[] = {0x05, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_LPTXTIMECNT, lptxtimecnt, sizeof(lptxtimecnt));
   static const uint8_t tclkheadercnt[] = {0x03, 0x1F, 0x00, 0x00};
   tc358779_wr_reg(TC_TCLK_HEADERCNT, tclkheadercnt, sizeof(tclkheadercnt));
   static const uint8_t tclktrailcnt[] = {0x02, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_TCLK_TRAILCNT, tclktrailcnt, sizeof(tclktrailcnt));
   static const uint8_t thsheadercnt[] = {0x04, 0x04, 0x00, 0x00};
   tc358779_wr_reg(TC_THS_HEADERCNT, thsheadercnt, sizeof(thsheadercnt));
   static const uint8_t twakeup[] = {0x00, 0x48, 0x00, 0x00};
   tc358779_wr_reg(TC_TWAKEUP, twakeup, sizeof(twakeup));
#endif /* USE_VIP */

   static const uint8_t tclkpostcnt[] = {0x0A, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_TCLK_POSTCNT, tclkpostcnt, sizeof(tclkpostcnt));
   static const uint8_t thstrailcnt[] = {0x04, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_THS_TRAILCNT, thstrailcnt, sizeof(thstrailcnt));
   // Enable the voltage regulators for the lanes
   static const uint8_t hstxvregen[] = {0x1F, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_HSTXVREGEN, hstxvregen, sizeof(hstxvregen));
   // Enable continuous clk mode
   static const uint8_t txoptioncntrl[] = {0x01, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_TXOPTIONCNTRL, txoptioncntrl, sizeof(txoptioncntrl));

   // Start the MIPI PPI
   static const uint8_t startcntrl[] = {0x01, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_STARTCNTRL, startcntrl, sizeof(startcntrl));

   // Start DSI Clock, which allows us to config the remaining registers
   static const uint8_t csidsistart[] = {0x01, 0x00, 0x00, 0x00};
   tc358779_wr_reg(TC_CSI_DSI_START, csidsistart, sizeof(csidsistart));

   // Don't halt on errors
   uint8_t timeout = 10;
   static const uint8_t dont_halt[] = {0xFF, 0xFF, 0x00, 0xC0 | 0x15};
   do {
      tc358779_wr_reg(TC_CSI_DSI_CONFW, dont_halt, sizeof(dont_halt));
   } while (!tc358779_confw_verify(dont_halt) && --timeout);
}

void tc358779_setup(void) {
   // Initilize I2C
   MX_I2C2_Init();
   LL_I2C_Enable(I2C2);

   // Enable TC358779_IRQ input
   LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);
   
   // Generate the EDID
   uint8_t edid[256] = {0};
   char uuid[13] = "HSK0000001";
   edid_generate(edid, 2023, 10, uuid);
   edid_generate_cea(edid + 128);

   // Disable audio? Reserved setting from Toshiba's initialization
   static const uint8_t confctl[] = {0x04, 0x00};
   tc358779_wr_reg(TC_CONFCTL, confctl, sizeof(confctl));

   // System reset?  Another reserved setting from Toshiba
   static const uint8_t full_reset[] = {0x00, 0x7F};
   tc358779_wr_reg(TC_SYSCTL, full_reset, sizeof(full_reset));
   LL_mDelay(10);
   // Clear reset and unsleep
   static const uint8_t clear_reset[] = {0x00, 0x00};
   tc358779_wr_reg(TC_SYSCTL, clear_reset, sizeof(clear_reset));
   LL_mDelay(1);
   // Clear the cached command, since it is gone
   g_h2c.command_len = 0;
   g_h2c.command_type = 0;

   // Start the PLLs
   // TODO: Only bump up the pll when going to high pixel clocks
   static const uint8_t pllctl0[] = {FBD & 0xFF, (FBD >> 8) | (PRD << 4)};
   tc358779_wr_reg(TC_PLLCTL0, pllctl0, sizeof(pllctl0));
   // 500 MHz - 1 GHz, enable clocks
   static const uint8_t pllctl1[] = {0x13, 0x02};
   tc358779_wr_reg(TC_PLLCTL1, pllctl1, sizeof(pllctl1));
   LL_mDelay(1);

   // Global config
   panel_cfg_t panel;
   panel_current_timing(&panel);

#if USE_VIP
   // Set the number of bytes per line
   uint16_t line_pixels = panel.h_active * 3;
   uint8_t vwcnt[] = {line_pixels, line_pixels >> 8};
   tc358779_wr_reg(TC_VWCNT, vwcnt, sizeof(vwcnt));

   // Set the FIFO to the middle of what the spreadsheet says is ok
   static const uint8_t fifoctl[] = {0x03, 0x00};
   tc358779_wr_reg(TC_FIFOCTL, fifoctl, sizeof(fifoctl));

   // Undocumented command from the scalar spreadsheet
   static const uint8_t waitvsync[] = {0x01, 0x00};
   tc358779_wr_reg(TC_WAIT_VSYNC, waitvsync, sizeof(waitvsync));

   // Use word count instead of HDMI Rx measurement
   static const uint8_t dbctl[] = {0x40, 0x00};
   tc358779_wr_reg(TC_DEBCTL, dbctl, sizeof(dbctl));

   // Setup the PLL for the scalar
   static const uint8_t pll11ctl0[] = {0x24, 0x14};
   tc358779_wr_reg(TC_PLL11CTL0, pll11ctl0, sizeof(pll11ctl0));

   static const uint8_t pll11ctl1[] = {0x08, 0x06};
   tc358779_wr_reg(TC_PLL11CTL1, pll11ctl1, sizeof(pll11ctl1));
#else  /* USE_VIP */
   // Set the FIFO to where there is not color separation at the end of the row
   static const uint8_t fifoctl[] = {0x0A, 0x00};
   tc358779_wr_reg(TC_FIFOCTL, fifoctl, sizeof(fifoctl));
#endif /* USE_VIP */

   // Configure the MIPI PPI and Tx
   tc358779_mipi_config();

   // Do the init here to match what Toshiba's procedure says.  re-init later
   // to get new timings if they changed
   tc358779_mipi_init(&panel);

   // VIP block is disabled by default, so don't configure it for now

   // Interrupt config
   // Undocumented write to make sync status work properly (and interrupt?)
   tc358779_wr_reg_byte(0x85AB, 0);
   // Just enable the HDMI RX Interrupt
   static const uint8_t intmask[] = {0xBF, 0x05};
   tc358779_wr_reg(TC_INTMASK, intmask, sizeof(intmask));
   // Within HDMI Rx, enable interrupt on sync change
   tc358779_wr_reg_byte(TC_MISC_INTM, 0x1D);

   // HDMI Rx Control
#if USE_MANUAL_HPDO
   // 27 MHz, but don't power PHY yet
   tc358779_wr_reg_byte(TC_PHY_CTL0, 0x00);
#else
   // Set 27 MHz, auto phy power
   tc358779_wr_reg_byte(TC_PHY_CTL0, 0x01);
#endif /* USE_MANUAL_HPDO */
   // Don't enable the PHY yet if we have manual control
   tc358779_wr_reg_byte(TC_PHY_EN, 0x3E);
   // Set 27 MHz again
   tc358779_wr_reg_byte(TC_SYS_FREQ0, 0x8C);
   tc358779_wr_reg_byte(TC_SYS_FREQ1, 0x0A);

   // Phy Auto Reset at 1600 us
   tc358779_wr_reg_byte(TC_PHY_CTL1, 0x80);
   // Phy set bias and squelch
   tc358779_wr_reg_byte(TC_PHY_BIAS, 0x40);
   tc358779_wr_reg_byte(TC_PHY_CSQ, 0x0A);

   // Set DDC5V detect to have 200 ms delay
   tc358779_wr_reg_byte(TC_DDC_CTL, 0x33);
#if USE_MANUAL_HPDO
   // Manual control of HPD
   tc358779_wr_reg_byte(TC_HPD_CTL, 0x00);
#else
   // Interlock HPD to DDC5V detect (delayed)
   tc358779_wr_reg_byte(TC_HPD_CTL, 0x10);
#endif /* USE_MANUAL_HPDO */
   // Enable analog section for audio // TODO: Is this necessary?
   tc358779_wr_reg_byte(TC_ANA_CTL, 0x31);
   // AVMute, not sure if we need this
   tc358779_wr_reg_byte(TC_AVM_CTL, 0x2D);

   // Use internal EDID at 100 kHz
   tc358779_wr_reg_byte(TC_EDID_MODE, 0x01);
   // Use one 256 byte block
   tc358779_wr_reg_byte(TC_EDID_LEN, 0x01);

   // Write the EDID into EEPROM
   for (uint16_t i = 0; i < 256; i += 8) {
      tc358779_wr_reg(0x8C00 + i, edid + i, 8);
   }

   // Configure HDCP to load keys
   tc358779_wr_reg_byte(0x85D1, 0x01);
   // KSV Auto Clear Mode
   tc358779_wr_reg_byte(TC_HDCP_MODE, 0x24);
   // EESS_Err auto-unAuth
   tc358779_wr_reg_byte(0x8563, 0x11);
   // Data Island Error auto-unauth
   tc358779_wr_reg_byte(0x8564, 0x0F);

   // Absorb Hsync jitter
   tc358779_wr_reg_byte(TC_VI_MODE, 0xFE);
   // Use 444
   tc358779_wr_reg_byte(TC_VOUT_SET, 0x00);
   tc358779_wr_reg_byte(TC_VI_REP, 0x00);
   // Undocumented write, V/H timing follows input?
   //    tc358779_wr_reg_byte(0x8571, 0x02);

#if USE_MANUAL_HPDO
   // Manually power on the PHY
   tc358779_wr_reg_byte(TC_PHY_EN, 0x3F);
   // Manually enable HPD
   tc358779_wr_reg_byte(TC_HPD_CTL, 0x01);
#endif /* USE_MANUAL_HPDO */

#if USE_VIP
   panel_cfg_t panel_native;
   panel_get_timing(0, &panel_native);
   tc358779_vip_init(&panel, &panel_native);
#endif /* USE_VIP */

   g_h2c.mipi_mode = MIPI_MODE_NONE;
}

void tc358779_power_off(void) {
   LL_GPIO_ResetOutputPin(TC_RST_GPIO_Port, TC_RST_Pin);
   LL_mDelay(10);

   LL_GPIO_ResetOutputPin(TC_STDBY_GPIO_Port, TC_STDBY_Pin);
   LL_GPIO_ResetOutputPin(TC_PWR_GPIO_Port, TC_PWR_Pin);
   LL_mDelay(10);
}

bool tc358779_version(uint32_t* version) {
   uint8_t id[2] = {0};
   tc358779_rd_reg(TC_CHIPID, id, sizeof(id));
   *version = id[0] | (id[1] << 8);

   return 1;
}

static uint8_t tc358779_sys_status(void) {
   // Read the HDMI Rx main status register
   uint8_t sys_status = 0;
   tc358779_rd_reg(TC_SYS_STATUS, &sys_status, sizeof(sys_status));
   return sys_status;
}

bool tc358779_cable_status(void) {
   // Is DDV power present
   return (tc358779_sys_status() & 0x01);
}

bool tc358779_hpa_status(void) {
   // Check what we're outputting for hot plug
   uint8_t hpd_ctl = 0;
   tc358779_rd_reg(TC_HPD_CTL, &hpd_ctl, sizeof(hpd_ctl));
   return hpd_ctl & 0x01;
}

bool tc358779_tmds_clock_status(void) {
   return (tc358779_sys_status() & 0x02);
}

bool tc358779_tmds_pll_status(void) {
   // Is the HDMI PHY PLL locked
   return (tc358779_sys_status() & 0x04);
}

void tc358779_hdmi_reset(void) {
   // Reset HDMI
   static const uint8_t hdmi_reset[] = {0x00, 0x01};
   tc358779_wr_reg(TC_SYSCTL, hdmi_reset, sizeof(hdmi_reset));
   LL_mDelay(1);
   // Clear reset
   static const uint8_t clear_reset[] = {0x00, 0x00};
   tc358779_wr_reg(TC_SYSCTL, clear_reset, sizeof(clear_reset));
}

void tc358779_hdmi_phy_suspend(bool suspend) {
   if (suspend) {
      // Go to manual PHY control with 27 MHz xtal
      tc358779_wr_reg_byte(TC_PHY_CTL0, 0x00);
      // Suspend the PHY
      tc358779_wr_reg_byte(TC_PHY_EN, 0x3E);
   } else {
      // Go back to auto PHY control with 27 MHz xtal
      tc358779_wr_reg_byte(TC_PHY_CTL0, 0x01);
   }
}

bool tc358779_hdmi_de_status(void) {
#if TC_DEBUG
   tc358779_hdmi_debug(0);
#endif /* TC_DEBUG */

   // Is HDMI DE locked
   return (tc358779_sys_status() & 0x08);
}

bool tc358779_hdmi_v_status(void) {
   return (tc358779_sys_status() & 0x80);
}

void tc358779_hdmi_v_clear(void) {
   // Clear the V sync changed interrupt.  This needs to be cleared before
   // the full system interrupt can be cleared
   tc358779_wr_reg_byte(TC_MISC_INT, 0x02);
   // Clear the general interrupt status
   static const uint8_t clearstatus[2] = {0xFF, 0xFF};
   tc358779_wr_reg(TC_INTSTATUS, clearstatus, sizeof(clearstatus));
}

uint16_t tc358779_h_active(void) {
   uint8_t h_size[2] = {0};
   // Undocumented H active registers
   tc358779_rd_reg(0x8582, h_size, sizeof(h_size));
   return h_size[0] | (h_size[1] << 8);
}

uint16_t tc358779_v_active(void) {
   uint8_t v_size[2] = {0};
   // Undocumented V active registers
   tc358779_rd_reg(0x8588, v_size, sizeof(v_size));
   return v_size[0] | (v_size[1] << 8);
}

void tc358779_free_run(bool state) {
   // Output from SRAM?
   uint8_t debctl[] = {state, 0x00};
   tc358779_wr_reg(TC_DEBCTL, debctl, sizeof(debctl));
}

void tc358779_mipi_init(const panel_cfg_t* panel) {
#if USE_PULSE
   // Set the back porch size when using pulse mode
   uint8_t dsi_vbp[] = {panel->vbp, (panel->vbp >> 8) & 0x03};
   tc358779_wr_reg(TC_DSI_VBPR, dsi_vbp, sizeof(dsi_vbp));

   uint16_t v_blank = panel->vsync;
#else
   // If we use non-burst with sync events, back porch is included
   uint16_t v_blank = panel->vsync + panel->vbp;
#endif /* USE_PULSE */
   uint8_t dsi_vsync[] = {v_blank, v_blank >> 8};
   tc358779_wr_reg(TC_DSI_VSW, dsi_vsync, sizeof(dsi_vsync));

   // Set the vertical active
   uint8_t dsi_active[] = {panel->v_active, panel->v_active >> 8};
   tc358779_wr_reg(TC_DSI_VACT, dsi_active, sizeof(dsi_active));

#if USE_PULSE
   // Set the back porch size when using pulse mode
   uint8_t dsi_hbp[] = {panel->hbp, panel->hbp >> 8};
   tc358779_wr_reg(TC_DSI_HBPR, dsi_hbp, sizeof(dsi_hbp));

   uint16_t h_blank = panel->hsync;
#else
   // If we use non-burst with sync events, back porch is included
   uint16_t h_blank = panel->hsync + panel->hbp;
#endif /* USE_PULSE */
   // float pclk = panel->h_total * panel->v_total * panel->refresh / 1000000.0F;
   // uint16_t hsw = lroundf((float)h_blank * dsiclk / pclk);
   // FIXME: The correct hsw causes corruption, implying timings are wrong
   // elsewhere
   //    uint8_t dsi_hsw[] = {hsw, hsw >> 8};
   uint8_t dsi_hsw[] = {h_blank, h_blank >> 8};
   tc358779_wr_reg(TC_DSI_HSW, dsi_hsw, sizeof(dsi_hsw));
}

void tc358779_mipi_reset(void) {
   // Reset MIPI
   static const uint8_t mipi_reset[] = {0x00, 0x02};
   tc358779_wr_reg(TC_SYSCTL, mipi_reset, sizeof(mipi_reset));
   LL_mDelay(1);
   // Clear reset
   static const uint8_t clear_reset[] = {0x00, 0x00};
   tc358779_wr_reg(TC_SYSCTL, clear_reset, sizeof(clear_reset));
}

void tc358779_video_mode(void) {
   // 4 lane HS setup
   // Set the bits to go to HS
   uint8_t timeout = 10;
   static const uint8_t hs_video[] = {0x86, 0x00, 0x00, 0xA0 | 0x03};
   do {
      tc358779_wr_reg(TC_CSI_DSI_CONFW, hs_video, sizeof(hs_video));
   } while (!tc358779_confw_verify(hs_video) && --timeout);

   // Clear the CSI bit
   timeout = 10;
   static const uint8_t hs_video_dsi[] = {0x00, 0x80, 0x00, 0xC0 | 0x03};
   do {
      tc358779_wr_reg(TC_CSI_DSI_CONFW, hs_video_dsi, sizeof(hs_video_dsi));
   } while (!tc358779_confw_verify(hs_video_dsi) && --timeout);

   // Complete HDMI initialization
   tc358779_wr_reg_byte(TC_INIT_END, 0x01);

   // Start TX, using sync events if we are not using pulses
   static const uint8_t start_tx[] = {0x01 | (!USE_PULSE << 1), 0x00};
   tc358779_wr_reg(TC_DSICTL, start_tx, sizeof(start_tx));

   // Enable the actual video TX buffer
   static const uint8_t enable_tx_buf[] = {0x17, 0x0C};
   tc358779_wr_reg(TC_CONFCTL, enable_tx_buf, sizeof(enable_tx_buf));

   g_h2c.mipi_mode = MIPI_MODE_VIDEO;
}

void tc358779_mipi_set_tx(bool state) {
   // Enable or disable transmission over MIPI
   uint8_t set_tx[] = {state ? (0x01 | (!USE_PULSE << 1)) : 0x00, 0x00};
   tc358779_wr_reg(TC_DSICTL, set_tx, sizeof(set_tx));
}

static void tc358779_generic_mode(void) {
   // FIXME: chris.cheng@taec.toshiba.com says going back to LP doesn't work
   //    // Clear the HS bits to go back to LP
   //    static uint8_t lp_mode[] = {0x80, 0x00, 0x00, 0xC3};
   //    do {
   //        tc358779_wr_reg(TC_CSI_DSI_CONFW, lp_mode, sizeof(lp_mode));
   //    } while (!tc358779_confw_verify(lp_mode));

   g_h2c.mipi_mode = MIPI_MODE_GENERIC;
}

static bool tc358779_wait_write(bool blocking) {
   // Wait for the write to complete before changing any registers
   uint8_t timeout_cnt = 0;
   // The write occurs during v blank, so wait a couple of frames
   while (timeout_cnt < 35) {
      timeout_cnt++;
      uint8_t dc_start = 0;
      tc358779_rd_reg(TC_DSICTL, &dc_start, 1);
      if (!(dc_start & 0x04)) {
         return 1;
      } else if (!blocking) {
         return 0;
      }
      LL_mDelay(1);
   }

   // If we timed out, the DSI block is likely hung, so reset it
   tc358779_mipi_reset();
   tc358779_mipi_config();
   panel_cfg_t panel;
   panel_current_timing(&panel);
   tc358779_mipi_init(&panel);
   tc358779_video_mode();

   return 0;
}

static bool tc358779_mipi_command(uint8_t data_type, uint8_t const* buf, uint8_t len, bool blocking) {
   // Wait for the write to complete before changing any registers
   if (!tc358779_wait_write(blocking))
      return 0;

   // If we're sending the same command repeatedly, we can save a ton of
   // time by just reusing the existing data in the command registers
   // and re-issuing a send command
   if ((data_type != g_h2c.command_type) || (len != g_h2c.command_len) || memcpy(g_h2c.command_buf, buf, len)) {
      uint8_t p_len;
      uint8_t pkt_type;
      // short or long packet type
      if (len > 2) {
         p_len = len;
         pkt_type = 0x40;
      } else {
         p_len = 0;  // Use 0 parameter len for short writes
         pkt_type = 0x10;
      }

      // Set the types for the command
      uint8_t cmd_type[] = {data_type, pkt_type};
      tc358779_wr_reg(TC_DCSCMD_TYPE, cmd_type, sizeof(cmd_type));

      // Set the length of the parameters
      uint8_t cmdlen[] = {p_len, p_len >> 8};
      tc358779_wr_reg(TC_DCSCMD_WC, cmdlen, sizeof(cmdlen));

      uint16_t reg = TC_DCSCMD_WDSTART;
      uint8_t i = 0;
      while (len > 1) {
         // Write the actual command
         tc358779_wr_reg(reg + i, buf + i, 2);
         len -= 2;
         i += 2;
      }

      // If we had an odd number, write the last byte padded
      if (len) {
         uint8_t last_byte[] = {buf[i], 0x00};
         tc358779_wr_reg(reg + i, last_byte, sizeof(last_byte));
      }

      // Cache the command so we can repeat it much more quickly
      if (len <= MAX_CACHED_COMMAND) {
         g_h2c.command_len = len;
         g_h2c.command_type = data_type;
         memcpy(g_h2c.command_buf, buf, len);
      }
   }

   // Actually perform the command
   static const uint8_t dsictl[] = {0x05, 0x00};
   tc358779_wr_reg(TC_DSICTL, dsictl, sizeof(dsictl));

   return 1;
}

static uint8_t tc358779_read_fifo_status(void) {
   uint8_t fifostatus = 0;
   tc358779_rd_reg(TC_CSI_DSI_STATUS, &fifostatus, 1);
   if (fifostatus & 0x80) {
      return 2;
   } else if (fifostatus & 0x20) {
      return 1;
   } else {
      return 0;
   }
}

static bool tc358779_mipi_read_data_type(uint8_t* data_type, uint8_t mode, uint8_t command_len) {
   // Get the actual DSI Packet Data ID for Processor -> Peripheral
   if (mode == MIPI_MODE_GENERIC_READ) {
      switch (command_len) {
         case 0:
            *data_type = MIPI_DATA_GENR0;
            break;
         case 1:
            *data_type = MIPI_DATA_GENR1;
            break;
         case 2:
            *data_type = MIPI_DATA_GENR2;
            break;
         default:
            return 0;
      }
   } else if (mode == MIPI_MODE_DCS_READ) {
      switch (command_len - 1) {
         case 0:
            *data_type = MIPI_DATA_DCSR;
            break;
         default:
            return 0;
      }
   } else {
      // Just handle reads
      return 0;
   }

   return 1;
}

static bool tc358779_mipi_read_return_type(uint8_t* return_type, uint8_t mode, uint8_t read_len) {
   // Get the actual DSI Packet Data ID for Peripheral -> Processor
   if (mode == MIPI_MODE_GENERIC_READ) {
      switch (read_len) {
         case 1:
            *return_type = 0x11;
            break;
         case 2:
            *return_type = 0x12;
            break;
         default:
            *return_type = 0x1A;
            break;
      }
   } else if (mode == MIPI_MODE_DCS_READ) {
      switch (read_len) {
         case 1:
            *return_type = 0x21;
            break;
         case 2:
            *return_type = 0x22;
            break;
         default:
            *return_type = 0x1C;
            break;
      }
   } else {
      // Just handle reads
      return 0;
   }

   return 1;
}

bool tc358779_mipi_read(uint8_t mode, uint8_t const* command, uint8_t command_len, uint8_t* buf, uint8_t* buf_len, bool blocking) {
   if (g_h2c.mipi_mode != MIPI_MODE_GENERIC)
      tc358779_generic_mode();

   uint8_t data_type = 0;
   if (!tc358779_mipi_read_data_type(&data_type, mode, command_len))
      return 0;

   // Only set the maximum return packet size if it's different from the last
   if (g_h2c.max_return != *buf_len) {
      // Wait for the write to complete before changing any registers
      if (!tc358779_wait_write(blocking))
         return 0;
      g_h2c.max_return = *buf_len;
      static const uint8_t smpr[] = {MIPI_DATA_SMPR, 0x10};
      tc358779_wr_reg(TC_DCSCMD_TYPE, smpr, sizeof(smpr));
      // Word count zero
      static const uint8_t cmdlen[] = {0x00, 0x00};
      tc358779_wr_reg(TC_DCSCMD_WC, cmdlen, sizeof(cmdlen));
      // Set the size of the read
      uint8_t read_len[] = {g_h2c.max_return, 0};
      tc358779_wr_reg(TC_DCSCMD_WDSTART, read_len, sizeof(read_len));
      // Perform the actual command
      static const uint8_t dsictl[] = {0x05, 0x00};
      tc358779_wr_reg(TC_DSICTL, dsictl, sizeof(dsictl));
   }

   // On blocking calls where we absolutely need the right data at the right
   // time, make sure to flush the rx first
   if (blocking) {
      // Empty the receive FIFO
      while (tc358779_read_fifo_status()) {
         uint8_t entry[4];
         tc358779_rd_reg(TC_DSICMD_RDFIFO, entry, sizeof(entry));
      }
   }

   if (!tc358779_mipi_command(data_type, command, command_len, blocking))
      return 0;

   if (!blocking) {
      return 1;
   }

   // TODO: wait for the interrupt asynchronously
   // Wait for the read to complete
   int16_t timeout = 200;
   while (timeout--) {
      if (tc358779_read_fifo_status())
         break;

      // Also give up if there was an error
      uint8_t rxerr[3] = {0x00};
      tc358779_rd_reg(TC_DSI_RXERR, rxerr, sizeof(rxerr));
      if (rxerr[2] & 0x18)
         return 0;
   }

   // Read the data from the FIFO
   return tc358779_mipi_complete_read(mode, command_len, buf, buf_len);
}

void tc358779_mipi_flush_read(void) {
   static const uint8_t reset_fifo[] = {0x10};
   tc358779_wr_reg(TC_DSI_RESET, reset_fifo, sizeof(reset_fifo));

   uint8_t entry[4] = {0x00};
   // Empty the read FIFO
   while (tc358779_read_fifo_status()) {
      tc358779_rd_reg(TC_DSICMD_RDFIFO, entry, sizeof(entry));
   }
}

bool tc358779_mipi_complete_read(uint8_t mode, uint8_t command_len, uint8_t* buf, uint8_t* buf_len) {
   // TODO: Distinguish pending reads from actual errors by return value?
   // Don't block on the complete
   if (!tc358779_read_fifo_status())
      return 0;

   uint8_t data_type = 0;
   if (!tc358779_mipi_read_return_type(&data_type, mode, *buf_len))
      return 0;

   // TODO: Check the ECC on the reads, #52
   // Short reads are a single FIFO entry
   if (*buf_len <= 2) {
      uint8_t entry[4] = {0x00};
      do {
         // Note that sometimes there is a false entry of zeroes ahead
         // of our real entry, so keep reading entries until we either
         // get the type we are looking for or the FIFO is empty
         tc358779_rd_reg(TC_DSICMD_RDFIFO, entry, sizeof(entry));
      } while ((entry[0] != data_type) && tc358779_read_fifo_status());

      // Fail if the data didn't match
      if (entry[0] != data_type)
         return 0;

      // Otherwise copy the actual payload bytes
      memcpy(buf, entry + 1, *buf_len);
   } else {
      // Long reads are multi FIFO entry, with the first being a header
      uint8_t remaining = *buf_len;
      bool read_header = 0;
      while (remaining) {
         // Read an entry from the read FIFO
         uint8_t entry[4] = {0x00};
         tc358779_rd_reg(TC_DSICMD_RDFIFO, entry, sizeof(entry));

         if (!read_header) {
            if (entry[0] == data_type)
               read_header = 1;
         } else {
            // The entries are padded, so only copy the valid bytes
            uint8_t valid_bytes = remaining > 4 ? 4 : remaining;
            memcpy(buf, entry, valid_bytes);
            remaining -= valid_bytes;
            buf += valid_bytes;
         }

         // If we still need more, check if there is another FIFO entry
         if (remaining) {
            if (!tc358779_read_fifo_status()) {
               // notify that the read buffer is short
               *buf_len = *buf_len - remaining;
               return 0;
            }
         }
      }
   }

   return 1;
}

bool tc358779_mipi_write(uint8_t mode, const uint8_t* buf, uint8_t len) {
   if (g_h2c.mipi_mode != MIPI_MODE_GENERIC)
      tc358779_generic_mode();

   uint8_t data_type = 0;

   // Get the actual DSI Packet Data ID
   if (mode == MIPI_MODE_GENERIC) {
      switch (len) {
         case 0:
            data_type = MIPI_DATA_GENSW0;
            break;
         case 1:
            data_type = MIPI_DATA_GENSW1;
            break;
         case 2:
            data_type = MIPI_DATA_GENSW2;
            break;
         default:
            data_type = MIPI_DATA_GENLW;
            break;
      }
   } else if (mode == MIPI_MODE_DCS) {
      // Subtract the command byte from the parameters for DCS
      switch (len - 1) {
         case 0:
            data_type = MIPI_DATA_DCSSW0;
            break;
         case 1:
            data_type = MIPI_DATA_DCSSW1;
            break;
         default:
            data_type = MIPI_DATA_DCSLW;
            break;
      }
   } else {
      // For now, just handle writes
      return 0;
   }

   if (tc358779_mipi_command(data_type, buf, len, 1)) {
      return 1;
   }

   return 0;
}

uint16_t tc358779_mipi_ack_err(void) {
   uint8_t status[] = {0x00, 0x00};
   tc358779_rd_reg(TC_DSI_ACKERR, status, sizeof(status));
   return status[0] | (status[1] << 8);
}

uint16_t tc358779_mipi_rx_err(void) {
   uint8_t status[] = {0x00, 0x00};
   tc358779_rd_reg(TC_DSI_RXERR, status, sizeof(status));
   return status[0] | (status[1] << 8);
}

uint16_t tc358779_mipi_dsi_err(void) {
   uint8_t status[] = {0x00, 0x00};
   tc358779_rd_reg(TC_CSI_DSI_ERR, status, sizeof(status));
   return status[0] | (status[1] << 8);
}

uint16_t tc358779_mipi_dsi_status(void) {
   uint8_t status[] = {0x00, 0x00};
   tc358779_rd_reg(TC_CSI_DSI_STATUS, status, sizeof(status));
   return status[0] | (status[1] << 8);
}

void tc3587789_hdmi_handle(void) {
   if (v_lock_changed) {
      v_lock_changed = 0;
      tc358779_hdmi_v_clear();
      if (tc358779_hdmi_v_status()) {
         // Only take action if the panel was powered off
         if (!is_panel_enabled()) {
            // If the resolution changed, we need to reconfigure
            // the HDMI receiver for the panel to work
            uint16_t h_active = tc358779_h_active();
            uint16_t v_active = tc358779_v_active();
            if (panel_resolution_changed(h_active, v_active)) {
               panel_cfg_t timing;
               // We don't have the actual refresh rate yet, so
               // get the default
               panel_get_timing(0, &timing);
               // Find a timing that matches the new resolution
               uint8_t index = panel_get_closest_timing(timing.refresh, h_active, v_active);
               // Get the timing and set it to the HDMI bridge
               panel_get_timing(index, &timing);
               tc358779_mipi_init(&timing);
            }

            // Don't power on panel until HDMI is present
            panel_power_on();

            // Repeat the panel enable until it succeeds, since it may
            // fail if TMDS briefly disappears
            while (tc358779_hdmi_v_status()) {
               // Switch from Amber to Blue when the screen goes active
               LL_GPIO_ResetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
               LL_GPIO_SetOutputPin(LED_Y_GPIO_Port, LED_Y_Pin);

               if (panel_enable(1)) {
                  break;
               } else {
                  // If a MIPI command fails, the H2C and/or panel are
                  // in an bad and unknown state, so we need to
                  // reset the TMDS PHY, the MIPI block (this happens
                  // in the tc358779_mipi_write), and the panel
                  tc358779_hdmi_phy_suspend(1);
                  panel_enable(0);
                  tc358779_mipi_set_tx(0);
                  tc358779_hdmi_phy_suspend(0);

                  LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
                  LL_GPIO_ResetOutputPin(LED_Y_GPIO_Port, LED_Y_Pin);

                  LL_mDelay(120);
               }
            }

            tc358779_mipi_set_tx(1);
            panel_set_brightness(brightness_set);
         }

      } else {
         // Sending MIPI commands while the HDMI PHY is active but there
         // is no input TMDS signal puts the H2C in a state where
         // all MIPI commands fail.  Powering down the HDMI PHY
         // before sending the panel off commands appears to function
         // as a workaround.
         tc358779_hdmi_phy_suspend(1);
         panel_enable(0);
         panel_power_off();
         tc358779_mipi_set_tx(0);
         // Turn the phy back to auto-on on cable detect to be prepared
         // for the HDMI signal coming back
         tc358779_hdmi_phy_suspend(0);

         // Switch from Blue to Amber when the screen goes to sleep
         LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
         LL_GPIO_ResetOutputPin(LED_Y_GPIO_Port, LED_Y_Pin);
      }
      //
      if (is_panel_enabled()) {
            uint16_t h_active = tc358779_h_active();
            uint16_t v_active = tc358779_v_active();
            panel_update_timing(60, h_active, v_active);
                
            // Set the right values on the bridge when the timings change
            panel_cfg_t new_timing;
            panel_current_timing(&new_timing);
            tc358779_mipi_init(&new_timing);
        }
   }
}
