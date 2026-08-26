// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Adapted from https://github.com/VladimirP1/esp-gyrologger (LGPL-2.1).
//
// The ESP32 I2C HAL does not expose a way to write the TX FIFO from an ISR
// without going through the (slower) register path, so upstream writes the FIFO
// via a hard-coded APB address. That address is port-specific: port 0 is
// 0x6001301C and port 1 is 0x6002701C (see I2C_DATA_APB_REG(i) in i2c_reg.h).
// We select the right one per port. The clock-gate / reset bits are likewise
// per-port (I2C_EXT0 vs I2C_EXT1 in dport_reg.h).

#include "esp_attr.h"
#include "hal/i2c_hal.h"
#include "soc/i2c_periph.h"

// APB address of the TX FIFO data register for a given I2C port.
// i2c_reg.h: #define I2C_DATA_APB_REG(i) (0x60013000 + (i) * 0x14000 + 0x001c)
static inline uint32_t mini_i2c_tx_fifo_addr(int i2c_num) {
    return 0x60013000 + (uint32_t)(i2c_num) * 0x14000u + 0x001cu;
}

static void IRAM_ATTR mini_i2c_write_txfifo(i2c_hal_context_t* hal, uint8_t* ptr, uint8_t len) {
    (void)hal;
    // The FIFO address is fixed per port; the port is known at init time and
    // stored in i2c_ctx (see mini_i2c.c). We read it from a global set there.
    extern int mini_i2c_port;
    uint32_t fifo_addr = mini_i2c_tx_fifo_addr(mini_i2c_port);
    for (int i = 0; i < len; i++) {
        WRITE_PERI_REG(fifo_addr, ptr[i]);
    }
}

#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#include "soc/system_reg.h"
#include "soc/dport_access.h"
static inline void mini_i2c_enable_hw(int i2c_num) {
    if (i2c_num == 0) {
        DPORT_SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT0_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT0_RST);
    } else {
        DPORT_SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT1_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT1_RST);
    }
}

static inline void mini_i2c_disable_hw(int i2c_num) {
    if (i2c_num == 0) {
        DPORT_CLEAR_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT0_CLK_EN);
        DPORT_SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT0_RST);
    } else {
        DPORT_CLEAR_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT1_CLK_EN);
        DPORT_SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT1_RST);
    }
}
#elif CONFIG_IDF_TARGET_ESP32
#include "soc/dport_reg.h"
static inline void mini_i2c_enable_hw(int i2c_num) {
    if (i2c_num == 0) {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT0_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT0_RST);
    } else {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT1_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT1_RST);
    }
}

static inline void mini_i2c_disable_hw(int i2c_num) {
    if (i2c_num == 0) {
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT0_CLK_EN);
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT0_RST);
    } else {
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT1_CLK_EN);
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT1_RST);
    }
}
#endif

void periph_module_enable(periph_module_t periph);
void periph_module_disable(periph_module_t periph);
