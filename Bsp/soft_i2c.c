/**
  ******************************************************************************
  * @file    soft_i2c.c
  * @brief   Software I2C implementation (from vendor CtrBoard-H7_ALL Bsp/sccb.c)
  ******************************************************************************
*/
#include "soft_i2c.h"
#include "dbg_print.h"

#define SI2C_SCL_H()  HAL_GPIO_WritePin(SI2C_SCL_PORT, SI2C_SCL_PIN, GPIO_PIN_SET)
#define SI2C_SCL_L()  HAL_GPIO_WritePin(SI2C_SCL_PORT, SI2C_SCL_PIN, GPIO_PIN_RESET)
#define SI2C_SDA_H()  HAL_GPIO_WritePin(SI2C_SDA_PORT, SI2C_SDA_PIN, GPIO_PIN_SET)
#define SI2C_SDA_L()  HAL_GPIO_WritePin(SI2C_SDA_PORT, SI2C_SDA_PIN, GPIO_PIN_RESET)
#define SI2C_SDA_IN() HAL_GPIO_ReadPin(SI2C_SDA_PORT, SI2C_SDA_PIN)

static void si2c_delay(void)
{
    volatile uint16_t i;
    for (i = 0; i < 8; i++) { __NOP(); }
}

void si2c_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /* open-drain, no pull (external pull-ups assumed on module) */
    g.Pin = SI2C_SCL_PIN | SI2C_SDA_PIN;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    SI2C_SCL_H();
    SI2C_SDA_H();
}

void si2c_start(void)
{
    SI2C_SDA_H(); SI2C_SCL_H(); si2c_delay();
    SI2C_SDA_L(); si2c_delay();
    SI2C_SCL_L(); si2c_delay();
}

void si2c_stop(void)
{
    SI2C_SCL_L(); SI2C_SDA_L(); si2c_delay();
    SI2C_SCL_H(); si2c_delay();
    SI2C_SDA_H(); si2c_delay();
}

uint8_t si2c_wait_ack(void)
{
    uint8_t ack;
    SI2C_SDA_H(); si2c_delay();
    SI2C_SCL_H(); si2c_delay();
    ack = (SI2C_SDA_IN() == GPIO_PIN_RESET) ? 0 : 1;  /* 0 = ACK */
    SI2C_SCL_L(); si2c_delay();
    return ack;
}

uint8_t si2c_send_byte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) SI2C_SDA_H(); else SI2C_SDA_L();
        si2c_delay();
        SI2C_SCL_H(); si2c_delay();
        SI2C_SCL_L();
        data <<= 1;
    }
    return si2c_wait_ack();
}

uint8_t si2c_read_byte(uint8_t ack)
{
    uint8_t data = 0, i;
    for (i = 0; i < 8; i++) {
        data <<= 1;
        SI2C_SCL_H(); si2c_delay();
        if (SI2C_SDA_IN() == GPIO_PIN_SET) data |= 1;
        SI2C_SCL_L(); si2c_delay();
    }
    if (ack == 0) SI2C_SDA_L(); else SI2C_SDA_H();  /* 0=ACK */
    si2c_delay();
    SI2C_SCL_H(); si2c_delay();
    SI2C_SCL_L();
    SI2C_SDA_H();
    return data;
}

uint8_t si2c_scan(void)
{
    uint8_t addr, found = 0;
    dbg_print("[I2C] scan start (SCL=PE8 SDA=PE7)\n");
    for (addr = 0x08; addr <= 0x77; addr++) {
        si2c_start();
        if (si2c_send_byte((uint8_t)(addr << 1)) == 0) {  /* ACK */
            dbg_print("[I2C] found 0x%02X\n", addr);
            found++;
        }
        si2c_stop();
    }
    dbg_print("[I2C] scan done, found=%d\n", found);
    return found;
}

void si2c_read_probe(uint8_t dev_addr_8bit, uint8_t nbytes)
{
    uint8_t i, d;
    si2c_start();
    if (si2c_send_byte(dev_addr_8bit | 0x01) != 0) {  /* read bit + no ACK */
        si2c_stop();
        dbg_print("[I2C] dev 0x%02X no ACK\n", dev_addr_8bit >> 1);
        return;
    }
    dbg_print("[I2C] dev 0x%02X data:", dev_addr_8bit >> 1);
    for (i = 0; i < nbytes; i++) {
        d = si2c_read_byte(i == (uint8_t)(nbytes - 1) ? 1 : 0);  /* last=NACK */
        dbg_print(" %02X", d);
    }
    dbg_print("\n");
    si2c_stop();
}
