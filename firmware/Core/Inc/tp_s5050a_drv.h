#ifndef _TP_S5050A_DRV_H_
#define _TP_S5050A_DRV_H_

#include <stdint.h>
#include <stdbool.h>

#define SYNAPTICS_RMI4_F01 0x01
#define SYNAPTICS_RMI4_F11 0x11
#define SYNAPTICS_RMI4_F12 0x12
#define SYNAPTICS_RMI4_F1A 0x1a
#define SYNAPTICS_RMI4_F34 0x34
#define SYNAPTICS_RMI4_F51 0x51
#define SYNAPTICS_RMI4_F54 0x54

void tp_init(void);
void tp_task(void);

#endif
