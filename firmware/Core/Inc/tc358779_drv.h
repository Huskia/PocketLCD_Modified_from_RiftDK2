#ifndef _TC358779_DRV_H_
#define _TC358779_DRV_H_

#include <stdint.h>
#include <stdbool.h>

// Global Configuration Registers
#define TC_CHIPID          0x0000
#define TC_SYSCTL          0x0002
#define TC_CONFCTL         0x0004
#define TC_FIFOCTL         0x0006
#define TC_AWCNT           0x0008
#define TC_VWCNT           0x000A
#define TC_PACKETID1       0x000C
#define TC_PACKETID2       0x000E
#define TC_PACKETID3       0x0010
#define TC_FCCTL           0x0012
#define TC_AUDFTPREM       0x001C
#define TC_SLMBCONFIG      0x001E
#define TC_PLLCTL0         0x0020
#define TC_PLLCTL1         0x0022
#define TC_PLL11CTL0       0x0024
#define TC_PLL11CTL1       0x0026
#define TC_WAIT_VSYNC      0x0060
#define TC_IOCTL0          0x0080
#define TC_IOCTL1          0x0082

// Interrupt Registers
#define TC_INTSTATUS       0x0014
#define TC_INTMASK         0x0016
#define TC_INTFLAG         0x0018
#define TC_INTSYSSTATUS    0x001A

// IR Registers

// CSI2/DSI TX PHY Registers
#define TC_CLW_CNTRL       0x0140
#define TC_D0W_CNTRL       0x0144
#define TC_D1W_CNTRL       0x0148
#define TC_D2W_CNTRL       0x014C
#define TC_D3W_CNTRL       0x0150

// CSI2/DSI TX PPI Registers
#define TC_STARTCNTRL      0x0204
#define TC_PPISTATUS       0x0208
#define TC_LINEINITCNT     0x0210
#define TC_LPTXTIMECNT     0x0214
#define TC_TCLK_HEADERCNT  0x0218
#define TC_TCLK_TRAILCNT   0x021C
#define TC_THS_HEADERCNT   0x0220
#define TC_TWAKEUP         0x0224
#define TC_TCLK_POSTCNT    0x0228
#define TC_THS_TRAILCNT    0x022C
#define TC_HSTXVREGCNT     0x0230
#define TC_HSTXVREGEN      0x0234
#define TC_TXOPTIONCNTRL   0x0238

// CSI2/DSI TX CTRL Registers
#define TC_CSI_DSI_CONTROL 0x040C
#define TC_CSI_DSI_STATUS  0x0410
#define TC_DSICMD_RDFIFO   0x0430
#define TC_DSI_ACKERR      0x0434
#define TC_DSI_RXERR       0x0440
#define TC_DSI_RXERR_HALT  0x0448
#define TC_CSI_DSI_ERR     0x044C
#define TC_CSI_DSI_ERR_HALT  0x0454
#define TC_CSI_DSI_CONFW   0x0500
#define TC_CS_DSI_LPCMD    0x0502
#define TC_DSI_RESET       0x0504
#define TC_CSI_DSI_START   0x0518

// CEC Registers

// DSI CTRL Registers
#define TC_DSICTL          0x0700
#define TC_DSI_VSW         0x0702
#define TC_DSI_VBPR        0x0704
#define TC_DSI_VACT        0x0706
#define TC_DSI_HSW         0x0708
#define TC_DSI_HBPR        0x070A
#define TC_DCSCMD_TYPE     0x070C
#define TC_DCSCMD_WC       0x070E
#define TC_DCSCMD_WDSTART  0x0710

// VIP Registers
#define TC_VBEMS_COM_TEST  0x4000

#define TC_CS_YHVSIN       0x5044
#define TC_CS_CHVSIN       0x5048
#define TC_CS_HSZOUT       0x504C
#define TC_CS_YHFILMODE    0x5050
#define TC_CS_YHFILPSMODE  0x5054
#define TC_CS_YMHFILBASE   0x505C
#define TC_CS_CHFILMODE    0x5070
#define TC_CS_CHFILPSMODE  0x5074
#define TC_CS_CMHFILBASE   0x507C
#define TC_CS_YVFILMODE    0x5090
#define TC_CS_YVFILPSMODE  0x5094
#define TC_CS_YVFILBASE    0x509C
#define TC_CS_CVFILMODE    0x50A0
#define TC_CS_CVFILPSMODE  0x50A4
#define TC_CS_CVFILBASE    0x50AC

#define TC_VIP_CONTROL     0x6000
#define TC_GO_LINES        0x6004
#define TC_VD_DELAY        0x6008
#define TC_VIP_VSW         0x600C
#define TC_VIP_VBP         0x6010
#define TC_VIP_VAL         0x6014
#define TC_VIP_VFP         0x6018
#define TC_VIP_HSW         0x601C
#define TC_VIP_HBP         0x6020
#define TC_VIP_HAP         0x6024
#define TC_VIP_HFP         0x6028
#define TC_VIP_VAS         0x602C
#define TC_VIP_DEBUG       0x6500
#define TC_VIP_FIFO_THRESHOLD  0x6504
#define TC_VIP_INIT_DELAY  0x650C
#define TC_VIP_FIFO_MAX    0x6514
#define TC_VIP_FIFO_MIN    0x6518
#define TC_VIP_FIFO_PIXEL  0x6524
#define TC_PP_FIFO_THRESHOLD   0x6508
#define TC_PP_FIFO_REACH   0x6510
#define TC_PP_FIFO_MAX     0x651C
#define TC_PP_FIFO_MIN     0x6520
#define TC_PP_FIFO_PIXEL   0x6528

// Internal Colorbar & Debug Registers
#define TC_DEBCTL          0x7080

// HDMI RX Interrupts Registers
#define TC_HDMI_INT0       0x8500
#define TC_CLK_INT         0x8503
#define TC_HDCP_INT        0x8508
#define TC_MISC_INT        0x850B
#define TC_KEY_INT         0x850F
#define TC_SYS_INTM        0x8512
#define TC_CLK_INTM        0x8513
#define TC_PACKET_INTM     0x8514
#define TC_HDCP_INTM       0x8518
#define TC_MISC_INTM       0x851B
#define TC_KEY_INTM        0x851F

// HDMI RX Status Registers
#define TC_SYS_STATUS      0x8520
#define TC_VI_STATUS       0x8521
#define TC_VI_STATUS1      0x8522
#define TC_VI_STATUS2      0x8525
#define TC_CLK_STATUS      0x8526
#define TC_PHYERR_STATUS   0x8527
#define TC_VI_STATUS3      0x8528

// HDMI RX Control Registers
#define TC_PHY_CTL0        0x8531
#define TC_PHY_CTL1        0x8532
#define TC_PHY_EN          0x8534
#define TC_PHY_RST         0x8535
#define TC_PHY_BIAS        0x8536
#define TC_PHY_CSQ         0x853F
#define TC_SYS_FREQ0       0x8540
#define TC_SYS_FREQ1       0x8541
#define TC_DDC_CTL         0x8543
#define TC_HPD_CTL         0x8544
#define TC_ANA_CTL         0x8545
#define TC_AVM_CTL         0x8546
#define TC_SOFT_RST        0x8547
#define TC_INIT_END        0x854A
#define TC_HDCP_MODE       0x8560
#define TC_VI_MODE         0x8570
#define TC_VOUT_SET        0x8573
#define TC_VI_REP          0x8576
#define TC_FH_MIN0         0x85AA
#define TC_HV_RST          0x85AF

#define TC_EDID_MODE       0x85C7
#define TC_EDID_LEN        0x85CB

// HDMI RX Audio Control Registers
#define TC_LKDET_FREQ      0x8630
#define TC_NCO_MOD         0x8670

enum {
    MIPI_MODE_NONE = 0,
    MIPI_MODE_GENERIC = 1,
    MIPI_MODE_DCS = 2,
    MIPI_MODE_VIDEO = 3,
    MIPI_MODE_GENERIC_READ = 4,
    MIPI_MODE_DCS_READ = 5,
};

enum {
    MIPI_DATA_VSTART    = 0x01,
    MIPI_DATA_VEND      = 0x11,
    MIPI_DATA_HSTART    = 0x21,
    MIPI_DATA_HEND      = 0x31,
    MIPI_DATA_EOT       = 0x08,
    MIPI_DATA_CMOFF     = 0x02,
    MIPI_DATA_CMON      = 0x12,
    MIPI_DATA_SHUTDOWN  = 0x22,
    MIPI_DATA_TURNON    = 0x32,
    MIPI_DATA_GENSW0    = 0x03,
    MIPI_DATA_GENSW1    = 0x13,
    MIPI_DATA_GENSW2    = 0x23,
    MIPI_DATA_GENR0     = 0x04,
    MIPI_DATA_GENR1     = 0x14,
    MIPI_DATA_GENR2     = 0x24,
    MIPI_DATA_DCSSW0    = 0x05,
    MIPI_DATA_DCSSW1    = 0x15,
    MIPI_DATA_DCSR      = 0x06,
    MIPI_DATA_SMPR      = 0x37,
    MIPI_DATA_NULL      = 0x09,
    MIPI_DATA_BLANK     = 0x19,
    MIPI_DATA_GENLW     = 0x29,
    MIPI_DATA_DCSLW     = 0x39,
    MIPI_DATA_RGB565    = 0x0E,
    MIPI_DATA_RGB666    = 0x1E,
    MIPI_DATA_RGB666L   = 0x2E,
    MIPI_DATA_RGB888    = 0x3E,
};

typedef struct {
   uint16_t h_active;
   uint16_t v_active;
   uint8_t hfp;
   uint8_t hbp;
   uint8_t hsync;
   uint8_t vfp;
   uint8_t vbp;
   uint8_t vsync;
   uint32_t h_total;
   uint32_t v_total;
   uint8_t refresh;
} panel_cfg_t;

void tc358779_power_on(void);
void tc358779_setup(void);
void tc358779_power_off(void);
bool tc358779_version(uint32_t *version);
bool tc358779_cable_status(void);
bool tc358779_hpa_status(void);
bool tc358779_tmds_clock_status(void);
bool tc358779_tmds_pll_status(void);
void tc358779_hdmi_reset(void);
void tc358779_hdmi_phy_suspend(bool suspend);
bool tc358779_hdmi_de_status(void);
bool tc358779_hdmi_v_status(void);
void tc358779_hdmi_v_clear(void);
uint16_t tc358779_h_active(void);
uint16_t tc358779_v_active(void);
void tc358779_mipi_init(const panel_cfg_t* panel);
void tc358779_mipi_reset(void);
void tc358779_video_mode(void);
void tc358779_mipi_set_tx(bool state);
bool tc358779_mipi_read(uint8_t mode, uint8_t const *command, uint8_t command_len, uint8_t *buf, uint8_t *buf_len, bool blocking);
void tc358779_mipi_flush_read(void);
bool tc358779_mipi_complete_read(uint8_t mode, uint8_t command_len, uint8_t *buf, uint8_t *buf_len);
bool tc358779_mipi_write(uint8_t mode, const uint8_t *buf, uint8_t len);
uint16_t tc358779_mipi_ack_err(void);
uint16_t tc358779_mipi_rx_err(void);
uint16_t tc358779_mipi_dsi_err(void);
uint16_t tc358779_mipi_dsi_status(void);
void tc3587789_hdmi_handle(void);

#endif
