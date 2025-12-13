#include <libopencm3/cm3/systick.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "fusb302.h"
#include "usart.h"

/*---- FUSB302 functions ----*/

void fusb_delay_ms(uint32_t ms) {
    // This assumes SysTick is running at 1ms intervals.
    for (uint32_t i = 0; i < ms; i++) {
        // Wait for the SysTick flag to be set (1ms elapsed)
        while ((STK_CSR & STK_CSR_COUNTFLAG) == 0);
    }
}

void fusb_reset(uint32_t i2c) {
    i2c_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(10);
}

void fusb_pd_reset(uint32_t i2c) {
    i2c_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_PD);
    fusb_delay_ms(10);
}

void fusb_full_reset(uint32_t i2c) {
    i2c_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW | FUSB302_RESET_PD);
    fusb_delay_ms(10);
}

void fusb_power_all(uint32_t i2c) {
    i2c_write_reg(i2c, FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(1);
}

uint8_t fusb_get_chip_id(uint32_t i2c) {
    uint8_t id = i2c_read_reg(i2c, FUSB302_REG_DEVICE_ID);
    return id;
}

int fusb_measure_cc_pin_src(uint32_t i2c, uint8_t cc_reg) {
    // Read status from switches0 register
    uint8_t reg, sw0_orig, cc_lvl;
    reg = i2c_read_reg(i2c, FUSB302_REG_SWITCHES0);
    sw0_orig = reg;
    // Clear measurement bits
    reg &= ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    // Set measurement bit for desired CC pin
    if (cc_reg == FUSB302_SW0_MEAS_CC1) {
        reg |= FUSB302_SW0_PU_EN1;  // Measure CC1
    } else if (cc_reg == FUSB302_SW0_MEAS_CC2) {
        reg |= FUSB302_SW0_PU_EN2;  // Measure CC2
    }
    // Set CC measure bit
    reg |= cc_reg;
    // Set measurement switch
    i2c_write_reg(i2c, FUSB302_REG_SWITCHES0, reg);
    // Set MDAC to default value
    uint8_t mdac = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    i2c_write_reg(i2c, FUSB302_REG_MEASURE, mdac);
    fusb_delay_ms(250);
    // Read status register
    reg = i2c_read_reg(i2c, FUSB302_REG_STATUS0);
    // Assume open
    cc_lvl = 0;
    // CC voltage below no connect threshold
    if ((reg & FUSB302_STATUS0_COMP) == 0) {
        i2c_write_reg(i2c, FUSB302_REG_MEASURE, PD_SRC_DEF_RD_MV);
        fusb_delay_ms(250);

        // Read status register
        reg = i2c_read_reg(i2c, FUSB302_REG_STATUS0);

        cc_lvl = (reg & FUSB302_STATUS0_COMP) ? TYPEC_CC_VOLT_RD : TYPEC_CC_VOLT_RA;
    }
    // Restore original switches0 register
    i2c_write_reg(i2c, FUSB302_REG_SWITCHES0, sw0_orig);
    return cc_lvl;
}

void fusb_enable_gcrc(uint32_t i2c, bool enable) {
    // AUTO_GCRC is in SWITCHES1 register
    uint8_t reg = i2c_read_reg(i2c, FUSB302_REG_SWITCHES1);
    if (enable) {
        reg |= FUSB302_SW1_AUTO_GCRC;
    } else {
        reg &= ~FUSB302_SW1_AUTO_GCRC;
    }
}

int fusb_check_cc_lines(int32_t i2c) {
    int ret = 0;
    fusb_power_all(i2c);
    int cc1_lvl = fusb_measure_cc_pin_src(i2c, FUSB302_SW0_MEAS_CC1);
    int cc2_lvl = fusb_measure_cc_pin_src(i2c, FUSB302_SW0_MEAS_CC2);
    if (cc1_lvl != TYPEC_CC_VOLT_OPEN && cc2_lvl == TYPEC_CC_VOLT_OPEN) {
        ret = 1; // Device detected on CC1
    } else if (cc2_lvl != TYPEC_CC_VOLT_OPEN && cc1_lvl == TYPEC_CC_VOLT_OPEN) {
        ret = 2; // Device detected on CC2
    }
    return ret;
}

void fusb_flush_rx(uint32_t i2c) {
    // Flush RX
    uint8_t reg = i2c_read_reg(i2c, FUSB302_REG_CONTROL1);
    reg |= FUSB302_CTL1_RX_FLUSH;
    i2c_write_reg(i2c, FUSB302_REG_CONTROL1, reg);
}

void fusb_flush_tx(uint32_t i2c) {
    // Flush TX
    uint8_t reg = (i2c, FUSB302_REG_CONTROL0);
    reg |= FUSB302_CTL0_TX_FLUSH;
    i2c_write_reg(i2c, FUSB302_REG_CONTROL0, reg);
}

void fusb_init_sink(uint32_t i2c) {
    // Reset FUSB302
    fusb_full_reset(i2c);
    // Power on all blocks
    fusb_power_all(i2c);
    // Auto-CRC, Use CC1 TX, Set Sink Role
    uint8_t switches1 = FUSB302_SW1_AUTO_GCRC | FUSB302_SW1_SPECREV1 | FUSB302_SW1_SPECREV0;
    i2c_write_reg(i2c, FUSB302_REG_SWITCHES1, switches1);
    // Toggle to detect CC and establish UFP (Sink)
    uint8_t control2 = FUSB302_CTL2_MODE_UFP | FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE;
    i2c_write_reg(i2c, FUSB302_REG_CONTROL2, control2);
    usart_printf("FUSB302 initialized in Sink mode.\n");
}

void fusb_setup_sniffer(int32_t i2c) {
    uint8_t reg, clear_mask;
    
    usart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
    // Reset the FUSB302
    i2c_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW | FUSB302_RESET_PD);
    fusb_delay_ms(200);

    // Power on
    i2c_write_reg(i2c, FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(200);

     // Configure Switches0: Enable measurement (passive detection) on CC1 and CC2
    uint8_t switches0 = FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2;
    i2c_write_reg(i2c, FUSB302_REG_SWITCHES0, switches0);

    // Disable pull-downs
    reg = i2c_read_reg(i2c, FUSB302_REG_SWITCHES0);
    clear_mask = ~(FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2) & 0xFF;
    reg &= clear_mask;
    i2c_write_reg(i2c, FUSB302_REG_SWITCHES0, reg);

    // Configure Control1: Enable reception of all SOP packet types for sniffing:
    // SOP', SOP'', SOP'_DEBUG, SOP''_DEBUG
    uint8_t control1 = FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB;
    i2c_write_reg(i2c, FUSB302_REG_CONTROL1, control1);

    // Configure Interrupt Masks: Unmask CRC_CHK (valid packet received) and ACTIVITY
    uint8_t mask = 0xFF; 
    mask &= ~(FUSB302_MASK_CRC_CHK | FUSB302_MASK_ACTIVITY);
    i2c_write_reg(i2c, FUSB302_REG_MASK, mask);

    // Unmask all interrupts 
    i2c_write_reg(i2c, FUSB302_REG_MASKA, 0x00);
    i2c_write_reg(i2c, FUSB302_REG_MASKB, 0x00);
    fusb_delay_ms(200);

    usart_printf("FUSB302 configured for PD Sniffing.\n");
}

bool fusb_rx_empty(uint32_t i2c) {
    return (i2c_read_reg(i2c, FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY);
}