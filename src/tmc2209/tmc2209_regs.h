/*
 * TMC2209 register map, the bit fields this driver touches, and the UART
 * datagram around them.
 *
 * Every value is quoted from the datasheet in docs/tmc2209.pdf (Rev. 1.08,
 * 2022-05-25): section 4 is the UART protocol, section 5 the register map.
 * Each line names the section and page it comes from.
 */
#pragma once

/* ---- register addresses ---- */
#define TMC2209_GCONF                     0x00        /* 5.1, p.23 */
#define TMC2209_GSTAT                     0x01        /* 5.1, p.24 */
#define TMC2209_IFCNT                     0x02        /* 5.1, p.24 */
#define TMC2209_SLAVECONF                 0x03        /* 5.1, p.24 */
#define TMC2209_IHOLD_IRUN                0x10        /* 5.2, p.28 */
#define TMC2209_TPOWERDOWN                0x11        /* 5.2, p.28 */
#define TMC2209_TSTEP                     0x12        /* 5.2, p.28 */
#define TMC2209_TPWMTHRS                  0x13        /* 5.2, p.28 */
#define TMC2209_TCOOLTHRS                 0x14        /* 5.3, p.29 */
#define TMC2209_VACTUAL                   0x22        /* 5.2, p.28 */
#define TMC2209_SGTHRS                    0x40        /* 5.3, p.29 */
#define TMC2209_SG_RESULT                 0x41        /* 5.3, p.29 */
#define TMC2209_CHOPCONF                  0x6C        /* 5.5.1, p.33 */
#define TMC2209_DRV_STATUS                0x6F        /* 5.5.3, p.37 */

/* Bit 7 of the register address byte: 1 writes, 0 reads. 4.1, p.18 */
#define TMC2209_WRITE                     0x80

/* ---- GCONF, 5.1 p.23 ---- */
#define TMC2209_GCONF_I_SCALE_ANALOG      (1u << 0)
#define TMC2209_GCONF_EN_SPREADCYCLE      (1u << 2)
#define TMC2209_GCONF_SHAFT               (1u << 3)
#define TMC2209_GCONF_PDN_DISABLE         (1u << 6)
#define TMC2209_GCONF_MSTEP_REG_SELECT    (1u << 7)

/* ---- GSTAT, 5.1 p.24: write 1 to clear ---- */
#define TMC2209_GSTAT_RESET               (1u << 0)
#define TMC2209_GSTAT_DRV_ERR             (1u << 1)
#define TMC2209_GSTAT_UV_CP               (1u << 2)

/* ---- SLAVECONF, 5.1 p.24 ---- */
#define TMC2209_SLAVECONF_SENDDELAY_SHIFT 8           /* bits 11..8 */

/* ---- IHOLD_IRUN, 5.2 p.28 ---- */
#define TMC2209_IHOLD_SHIFT               0           /* bits 4..0 */
#define TMC2209_IRUN_SHIFT                8           /* bits 12..8 */
#define TMC2209_IHOLDDELAY_SHIFT          16          /* bits 19..16 */

/* ---- CHOPCONF, 5.5.1 p.33-34 ---- */
#define TMC2209_CHOP_TOFF_MASK            0x0000000Fu /* bits 3..0; 0: off */
#define TMC2209_CHOP_VSENSE               (1u << 17)
#define TMC2209_CHOP_MRES_SHIFT           24          /* bits 27..24 */
#define TMC2209_CHOP_MRES_MASK            (0xFu << TMC2209_CHOP_MRES_SHIFT)
#define TMC2209_CHOP_INTPOL               (1u << 28)

/* ---- DRV_STATUS, 5.5.3 p.37 ---- */
#define TMC2209_DRV_OTPW                  (1u << 0)
#define TMC2209_DRV_OT                    (1u << 1)
#define TMC2209_DRV_S2GA                  (1u << 2)
#define TMC2209_DRV_S2GB                  (1u << 3)
#define TMC2209_DRV_S2VSA                 (1u << 4)   /* low side short */
#define TMC2209_DRV_S2VSB                 (1u << 5)
#define TMC2209_DRV_OLA                   (1u << 6)
#define TMC2209_DRV_OLB                   (1u << 7)
#define TMC2209_DRV_CS_ACTUAL_SHIFT       16          /* bits 20..16 */
#define TMC2209_DRV_CS_ACTUAL_MASK        (0x1Fu << TMC2209_DRV_CS_ACTUAL_SHIFT)
#define TMC2209_DRV_STEALTH               (1u << 30)
#define TMC2209_DRV_STST                  (1u << 31)

/* ---- SG_RESULT, 5.3 p.29: 10 bits ---- */
#define TMC2209_SG_RESULT_MASK            0x03FFu

/* ---- UART datagrams, 4.1 p.18-19 ----
 * Sync is the nibble 1010 sent LSB first, which puts 0x05 on the wire; a
 * reply goes to master address 0xFF. A write is 8 bytes, a read request 4,
 * its reply 8. */
#define TMC2209_SYNC                      0x05
#define TMC2209_MASTER_ADDR               0xFF
#define TMC2209_WRITE_LEN                 8
#define TMC2209_READ_REQ_LEN              4
#define TMC2209_REPLY_LEN                 8
#define TMC2209_MAX_SLAVE_ADDR            3           /* MS1/MS2 strap */
