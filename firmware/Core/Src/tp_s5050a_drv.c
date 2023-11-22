/************************************************
 * A brief driver for touch screen with driver ic
 * S5050A, only comfirmed on FPC "TSP Rev 3.4"
 ************************************************/
#include "tp_s5050a_drv.h"
#include <stdio.h>
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"

#if 0
#define TP_LOG(format, args...) \
   printf(format, ##args);
#else
#define TP_LOG(format, ...) ((void)0)
#endif

#define S5050A_SLA 0x20

uint8_t tp_attacthed = 0;

typedef struct {
   uint16_t confidence;  // each bit indicates a finger
   uint8_t number;       // total fingers in one touch event (first finger touched until the last finger release)
   uint16_t x_pos[10];   // coordinate of each finger
   uint16_t y_pos[10];
} tp_position;

/***************************************************************************************/
static void synaptics_rmi4_write(uint8_t reg, uint8_t* data, uint8_t len) {
   i2c_master_tx(I2C1, S5050A_SLA, &reg, 1, (uint8_t*)data, len);
}

static void synaptics_rmi4_write_byte(uint8_t reg, uint8_t data) {
   i2c_master_tx(I2C1, S5050A_SLA, &reg, 1, (uint8_t*)&data, 1);
}

static void synaptics_rmi4_read(uint8_t reg, uint8_t* data, uint8_t len) {
   i2c_master_rx(I2C1, S5050A_SLA, &reg, 1, data, len);
}

void delay_us(uint32_t us) {
   uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
   while (delay--) {
      ;
   }
}

/***************************************************************************************/
static void synaptics_rmi4_check_status(void) {
   uint8_t rmi_stat = 0;
   synaptics_rmi4_read(0x04, &rmi_stat, 1);
   TP_LOG("tp stat 04: %02X\n", rmi_stat);

   synaptics_rmi4_read(0x05, &rmi_stat, 1);
   TP_LOG("tp stat 05: %02X\n", rmi_stat);
}

static void synaptics_rmi4_f12_init(void) {
   uint8_t rmi_data[20];
   synaptics_rmi4_read(0x42, rmi_data, 5);
   synaptics_rmi4_read(0x18, rmi_data, 2);
   synaptics_rmi4_read(0x44, rmi_data, 1);
   synaptics_rmi4_read(0x45, rmi_data, 3);
   synaptics_rmi4_read(0x47, rmi_data, 1);
   synaptics_rmi4_read(0x19, rmi_data, 1);
   delay_us(90);
   synaptics_rmi4_write_byte(0x19, 0x00);
   delay_us(90);
   synaptics_rmi4_write_byte(0x1A, 0xDF);
   delay_us(90);
   synaptics_rmi4_read(0x11, rmi_data, 14);
   synaptics_rmi4_read(0x12, rmi_data, 16);
   synaptics_rmi4_read(0x14, rmi_data, 10);
}

static void synaptics_rmi4_f34_init(void) {
   uint8_t rmi_data[4];
   synaptics_rmi4_read(0x1C, rmi_data, 4);
   synaptics_rmi4_read(0x08, rmi_data, 4);
   TP_LOG("tp version: %02X\n", rmi_data[3]);
}

static void synaptics_rmi4_f51_init(void) {
   uint8_t rmi_register[6];
   synaptics_rmi4_read(0x49, rmi_register, 6);
   synaptics_rmi4_write_byte(0x16, 0x82);
   delay_us(90);
}

/***************************************************************************************/
void tp_init(void) {
   uint8_t page_number, pdt_entry_addr;
   uint8_t rmi_fd[6] = {0};
   for (page_number = 0; page_number < 10; page_number++) {
      synaptics_rmi4_write_byte(0xFF, page_number);
      delay_us(90);
      for (pdt_entry_addr = 0xE9; pdt_entry_addr > 0x0A; pdt_entry_addr -= 0x06) {
         synaptics_rmi4_read(pdt_entry_addr, rmi_fd, 6);
         if (rmi_fd[5] == 0x00) {
            break;
         }
         switch (rmi_fd[5]) {
            case SYNAPTICS_RMI4_F01:
               synaptics_rmi4_check_status();
               break;
            case SYNAPTICS_RMI4_F11:  // this addr not exist
               break;
            case SYNAPTICS_RMI4_F12:
               synaptics_rmi4_f12_init();
               break;
            case SYNAPTICS_RMI4_F1A:  // this addr not exist
               break;
            case SYNAPTICS_RMI4_F34:
               synaptics_rmi4_f34_init();
               break;
            case SYNAPTICS_RMI4_F51:
               synaptics_rmi4_f51_init();
               break;
            case SYNAPTICS_RMI4_F54:
               break;
         }
      }
   }

   synaptics_rmi4_write_byte(0xFF, 0x00);  // page 0
   delay_us(90);
   synaptics_rmi4_read(0x05, rmi_fd, 1);
   // initilaze finish
   LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_5);  // Enable S5050A_IRQ input
   tp_attacthed = 0;
}

int tp_get_points(tp_position* tp_pos) {
   uint8_t tp_num = 0;
   uint16_t tp_conf = 0;
   uint8_t rmi_data[80] = {0};
   synaptics_rmi4_write_byte(0xFF, 0x00);  // page 0
   delay_us(50);
   synaptics_rmi4_read(0x04, rmi_data, 2);  // get stat
   delay_us(50);
   TP_LOG("tp stat: %02X, ", rmi_data[1]);

   synaptics_rmi4_read(0x07, rmi_data, 2);  // get points
   tp_pos->confidence = tp_conf = rmi_data[0] | (rmi_data[1] << 8);
   TP_LOG("tp fingers: %04X\n", tp_pos->confidence);
   for (uint8_t i = 6; i < 16; i++) {  // get number
      if ((tp_conf << i) & 0x8000) {
         tp_num = 16 - i;
         tp_pos->number = tp_num;
         break;
      }
   }

   if (tp_num > 0) {
      synaptics_rmi4_read(0x06, rmi_data, tp_num * 7);  // get xy data
   } else {
      synaptics_rmi4_read(0x06, rmi_data, 7);  // if last finger leave, tm_num will be 0, but must read atleast one point, otherwise the chip will not response
   }
   for (uint8_t i = 0; i < tp_num; i++) {
      TP_LOG("tp %d: %02X%02X%02X%02X%02X%02X%02X\n", i, rmi_data[i * 7], rmi_data[i * 7 + 1], rmi_data[i * 7 + 2], rmi_data[i * 7 + 3], rmi_data[i * 7 + 4], rmi_data[i * 7 + 5], rmi_data[i * 7 + 6]);
      TP_LOG("tp %d: x: %d, y: %d\n", i, (rmi_data[i * 7 + 1] | rmi_data[i * 7 + 2] << 8), (rmi_data[i * 7 + 3] | rmi_data[i * 7 + 4] << 8));
      tp_pos->x_pos[i] = rmi_data[i * 7 + 1] | rmi_data[i * 7 + 2] << 8;
      tp_pos->y_pos[i] = rmi_data[i * 7 + 3] | rmi_data[i * 7 + 4] << 8;
   }

   // synaptics_rmi4_write_byte(0xFF, 0x04);  // page 4
   // delay_us(90);
   // synaptics_rmi4_read(0x00, rmi_data, 7);
   // delay_us(90);
   // synaptics_rmi4_read(0x09, rmi_data, 9);

   return tp_num;
}

/* Report touch point(s) to USB */
static tp_position tp_pos;
static uint8_t usb_send_buf[52] = {0};

void tp_task(void) {
   if (tp_attacthed) {
      tp_attacthed = 0;
      tp_get_points(&tp_pos);

      // organize usb package
      usb_send_buf[0] = 0x01;  // report ID
      for (uint8_t i = 0; i < 10; i++) {
         usb_send_buf[1 + i * 5] = (tp_pos.confidence & (0x0001 << i)) ? (0xC0 + i) : (0x00 + i);
         usb_send_buf[1 + i * 5 + 1] = tp_pos.x_pos[i] & 0xFF;
         usb_send_buf[1 + i * 5 + 2] = tp_pos.x_pos[i] >> 8;
         usb_send_buf[1 + i * 5 + 3] = tp_pos.y_pos[i] & 0xFF;
         usb_send_buf[1 + i * 5 + 4] = tp_pos.y_pos[i] >> 8;
      }
      usb_send_buf[51] = tp_pos.number;
      usb_device_hid_send(usb_send_buf, 52);
   }
}
