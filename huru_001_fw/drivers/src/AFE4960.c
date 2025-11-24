#include "AFE4960.h"
#include "AFE4960_ex.h"
#include "hal.h"
#include "board_config.h"
#include "nrf_log.h"
#include "epoch.h"

int32_t AFE4960_write_reg(uint8_t addr, uint32_t value)
{
    int32_t err;
    uint8_t data[3];
    data[0] = (uint8_t)((value >> 16) & 0x000000ff);
    data[1] = (uint8_t)((value >> 8) & 0x000000ff);
    data[2] = (uint8_t)((value >> 0) & 0x000000ff);
    err = hal_i2c_write(AFE_I2C_BUS, AFE_I2C_ADD, addr, data, 3);
    return err;
}

int32_t AFE4960_read_reg(uint8_t addr, uint32_t *value)
{
    int32_t err;
    uint8_t data[3];
    err = hal_i2c_read(AFE_I2C_BUS, AFE_I2C_ADD, addr, data, 3);
    *value = (data[0] << 16) | (data[1] << 8) | (data[2]);
    return err;
}