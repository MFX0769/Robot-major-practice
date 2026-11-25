/**
  ******************************************************************************
  * @file    soft_i2c.h
  * @brief   Software I2C (bit-bang) based on vendor SCCB (2026-11-22)
  * @note    Line-sensor experiment: MCU TX=SCL=PE8, MCU RX=SDA=PE7
  *          (user rewired U7 pins as I2C). Open-drain + external pull-ups.
  ******************************************************************************
*/
#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* pins: SCL = PE8 (was UART7_TX), SDA = PE7 (was UART7_RX) */
#define SI2C_SCL_PORT   GPIOE
#define SI2C_SCL_PIN    GPIO_PIN_8
#define SI2C_SDA_PORT   GPIOE
#define SI2C_SDA_PIN    GPIO_PIN_7

void si2c_init(void);
void si2c_start(void);
void si2c_stop(void);
uint8_t si2c_wait_ack(void);
uint8_t si2c_send_byte(uint8_t data);
uint8_t si2c_read_byte(uint8_t ack);

/* scan all 7-bit addresses (0x08..0x77), print found ones via debug; returns count */
uint8_t si2c_scan(void);

/* read N bytes from a (7-bit<<1) device address, print hex via debug */
void si2c_read_probe(uint8_t dev_addr_8bit, uint8_t nbytes);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_I2C_H */
