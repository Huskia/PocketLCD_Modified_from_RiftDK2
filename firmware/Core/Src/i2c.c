/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableClockStretching(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableClockStretching(I2C2);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/* USER CODE BEGIN 1 */
int8_t i2c_master_tx(I2C_TypeDef* I2Cx, uint8_t sla, uint8_t* reg, uint8_t reg_len, uint8_t* buf, uint8_t buf_len) {
   /* (1) Prepare acknowledge for Master data reception ************************/
   LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

   /* (2) Initiate a Start condition to the Slave device ***********************/
   /* Master Generate Start condition */
   LL_I2C_GenerateStartCondition(I2Cx);

   /* (3) Loop until Start Bit transmitted (SB flag raised) ********************/
   uint32_t ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_SB(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (4) Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request */
   LL_I2C_TransmitData8(I2Cx, sla << 1);

   /* (5) Loop until Address Acknowledgement received (ADDR flag raised) *******/
   ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_ADDR(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (6) Clear ADDR flag and loop until end of transfer (ubNbDataToTransmit == 0) */
   LL_I2C_ClearFlag_ADDR(I2Cx);

   // 6.1 Check TXE flag & write reg
   for (int i = 0; i < reg_len; i++) {
      ot_cnt = 0;
      while (!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
         if (ot_cnt++ > 72000)
            return -1;
      }
      LL_I2C_TransmitData8(I2Cx, reg[i]);
   }
   // 6.2 Check TXE flag & write buff
   for (int i = 0; i < buf_len; i++) {
      ot_cnt = 0;
      while (!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
         if (ot_cnt++ > 72000)
            return -1;
      }
      LL_I2C_TransmitData8(I2Cx, buf[i]);
   }

   ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (7) End of transfer, Data consistency are checking into Slave process *****/
   LL_I2C_GenerateStopCondition(I2Cx);
   return 0;
}

int8_t i2c_master_rx(I2C_TypeDef* I2Cx, uint8_t sla, uint8_t* reg, uint8_t reg_len, uint8_t* buf, uint8_t buf_len) {
   /* (1) Prepare acknowledge for Master data reception ************************/
   LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

   /* (2) Initiate a Start condition to the Slave device ***********************/
   /* Master Generate Start condition */
   LL_I2C_GenerateStartCondition(I2Cx);

   /* (3) Loop until Start Bit transmitted (SB flag raised) ********************/
   uint32_t ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_SB(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (4) Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request */
   LL_I2C_TransmitData8(I2Cx, sla << 1);

   /* (5) Loop until Address Acknowledgement received (ADDR flag raised) *******/
   ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_ADDR(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (6) Clear ADDR flag and loop until end of transfer (ubNbDataToTransmit == 0) */
   LL_I2C_ClearFlag_ADDR(I2Cx);

   // 6.1 Check TXE flag & write reg
   for (int i = 0; i < reg_len; i++) {
      ot_cnt = 0;
      while (!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
         if (ot_cnt++ > 72000)
            return -1;
      }
      LL_I2C_TransmitData8(I2Cx, reg[i]);
   }

   // 6.2 waite last byte write finish
   ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (7) restart condition */
   LL_I2C_GenerateStartCondition(I2Cx);

   /* (8) check start bit flag */
   ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_SB(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (9) write device address (READ) */
   LL_I2C_TransmitData8(I2Cx, (sla << 1) | 0x01);

   /* (10) wait address sent */
   ot_cnt = 0;
   while (!LL_I2C_IsActiveFlag_ADDR(I2Cx)) {
      if (ot_cnt++ > 72000)
         return -1;
   }

   /* (11) clear ADDR flag */
   LL_I2C_ClearFlag_ADDR(I2Cx);

   // 11.1 check RXNE flag & read data
   for (int i = 0; i < buf_len; i++) {
      // 11.2 NACK at last byte
      if (i == buf_len - 1) {
         LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
         // 11.3 stop condition
         LL_I2C_GenerateStopCondition(I2Cx);
      } else {
         LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
      }

      ot_cnt = 0;
      while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
         if (ot_cnt++ > 72000)
            return -1;
      }

      buf[i] = LL_I2C_ReceiveData8(I2Cx);
   }

   return 0;
}

/* USER CODE END 1 */
