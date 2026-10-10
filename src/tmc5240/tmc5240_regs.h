/*
 * TMC5240 register addresses and the bit fields this driver touches.
 *
 * Every value is quoted from the datasheet in docs/tmc5240.pdf. Addresses
 * are those of Table 27, "Overview of Register Map" (p.81-83), and of each
 * register's own entry, whose page is given; bit positions come from that
 * entry. The names are esp-tmc5240-stepper's, which followed ADI's TMC-API:
 * INP_OUT is the datasheet's IOIN, SWMODE its SW_MODE, RAMPSTAT its
 * RAMP_STAT, XENC its X_ENC, DRVSTATUS its DRV_STATUS, GLOBAL_SCALER its
 * GLOBAL SCALER.
 *
 * Registers the driver never touches (MSLUT*, OTP, DCCTRL, virtual stops) are
 * left out on purpose. Reach them with tmc5240_read()/tmc5240_write().
 */
#pragma once

/* ---- register addresses: Table 27, and the entry's page ---- */
#define TMC5240_GCONF           0x00    /* p.90 */
#define TMC5240_GSTAT           0x01    /* p.92 */
#define TMC5240_IFCNT           0x02    /* p.93 */
#define TMC5240_INP_OUT         0x04    /* IOIN, p.94 */
#define TMC5240_DRV_CONF        0x0A    /* p.97 */
#define TMC5240_GLOBAL_SCALER   0x0B    /* p.98 */
#define TMC5240_IHOLD_IRUN      0x10    /* p.99 */
#define TMC5240_TPOWERDOWN      0x11    /* p.100 */
#define TMC5240_TSTEP           0x12    /* p.100 */
#define TMC5240_TPWMTHRS        0x13    /* p.101 */
#define TMC5240_TCOOLTHRS       0x14    /* p.102 */
#define TMC5240_THIGH           0x15    /* p.103 */
#define TMC5240_RAMPMODE        0x20    /* p.104 */
#define TMC5240_XACTUAL         0x21    /* p.105 */
#define TMC5240_VACTUAL         0x22    /* p.105 */
#define TMC5240_VSTART          0x23    /* p.106 */
#define TMC5240_A1              0x24    /* p.107 */
#define TMC5240_V1              0x25    /* p.107 */
#define TMC5240_AMAX            0x26    /* p.108 */
#define TMC5240_VMAX            0x27    /* p.109 */
#define TMC5240_DMAX            0x28    /* p.109 */
#define TMC5240_TVMAX           0x29    /* p.110 */
#define TMC5240_D1              0x2A    /* p.111 */
#define TMC5240_VSTOP           0x2B    /* p.112 */
#define TMC5240_TZEROWAIT       0x2C    /* p.112 */
#define TMC5240_XTARGET         0x2D    /* p.113 */
#define TMC5240_V2              0x2E    /* p.114 */
#define TMC5240_A2              0x2F    /* p.115 */
#define TMC5240_D2              0x30    /* p.115 */
#define TMC5240_VDCMIN          0x33    /* p.116 */
#define TMC5240_SWMODE          0x34    /* SW_MODE, p.117 */
#define TMC5240_RAMPSTAT        0x35    /* RAMP_STAT, p.119 */
#define TMC5240_XLATCH          0x36    /* p.121 */
#define TMC5240_ENCMODE         0x38    /* p.121 */
#define TMC5240_XENC            0x39    /* X_ENC, p.123 */
#define TMC5240_ENC_CONST       0x3A    /* p.123 */
#define TMC5240_ENC_STATUS      0x3B    /* p.124 */
#define TMC5240_ENC_LATCH       0x3C    /* p.125 */
#define TMC5240_ADC_VSUPPLY_AIN 0x50    /* p.128 */
#define TMC5240_ADC_TEMP        0x51    /* p.128 */
#define TMC5240_OTW_OV_VTH      0x52    /* p.129 */
#define TMC5240_MSCNT           0x6A    /* p.140 */
#define TMC5240_CHOPCONF        0x6C    /* p.141 */
#define TMC5240_COOLCONF        0x6D    /* p.144 */
#define TMC5240_DRVSTATUS       0x6F    /* DRV_STATUS, p.146 */
#define TMC5240_PWMCONF         0x70    /* p.148 */
#define TMC5240_PWM_SCALE       0x71    /* p.151 */
#define TMC5240_SG4_THRS        0x74    /* p.153 */
#define TMC5240_SG4_RESULT      0x75    /* p.153 */

/* Bit 39 of the datagram, write_not_read: OR into the address to write. */
#define TMC5240_WRITE           0x80

/* ---- INP_OUT (IOIN, 0x04), read only, p.94-95 ---- */
#define TMC5240_VERSION_SHIFT   24
#define TMC5240_VERSION_MASK    0xFFu
#define TMC5240_VERSION_FIRST   0x40   /* "0x40 = first version of the IC" */
#define TMC5240_EXT_CLK         (1u << 14)
#define TMC5240_ADC_ERR         (1u << 15)

/* ---- GCONF (0x00), p.90 ---- */
#define TMC5240_EN_PWM_MODE     (1u << 2)   /* StealthChop2 */
#define TMC5240_SHAFT           (1u << 4)   /* invert motor direction */

/* ---- GSTAT (0x01), p.92: write 1 to clear ---- */
#define TMC5240_RESET_FLAG      (1u << 0)
#define TMC5240_DRV_ERR         (1u << 1)
#define TMC5240_UV_CP           (1u << 2)

/* ---- DRV_CONF (0x0A), p.97-98 ---- */
#define TMC5240_CURRENT_RANGE_MASK 0x3u    /* 0:1A 1:2A 2:3A 3:3A, peak */

/* ---- IHOLD_IRUN (0x10), p.99 ---- */
#define TMC5240_IHOLD_SHIFT      0
#define TMC5240_IRUN_SHIFT       8
#define TMC5240_IHOLDDELAY_SHIFT 16
#define TMC5240_IRUNDELAY_SHIFT  24

/* ---- RAMPMODE (0x20), p.104 ---- */
#define TMC5240_MODE_POSITION   0
#define TMC5240_MODE_VELPOS     1   /* velocity mode toward +VMAX */
#define TMC5240_MODE_VELNEG     2   /* velocity mode toward -VMAX */
#define TMC5240_MODE_HOLD       3

/* ---- RAMPSTAT (RAMP_STAT, 0x35), p.119-120 ---- */
#define TMC5240_STATUS_STOP_L      (1u << 0)
#define TMC5240_STATUS_STOP_R      (1u << 1)
#define TMC5240_EVENT_STOP_L       (1u << 4)
#define TMC5240_EVENT_STOP_R       (1u << 5)
#define TMC5240_EVENT_STOP_SG      (1u << 6)
#define TMC5240_EVENT_POS_REACHED  (1u << 7)
#define TMC5240_VELOCITY_REACHED   (1u << 8)
#define TMC5240_POSITION_REACHED   (1u << 9)
#define TMC5240_VZERO              (1u << 10)

/* ---- CHOPCONF (0x6C), p.141 ---- */
#define TMC5240_TOFF_SHIFT      0
#define TMC5240_HSTRT_SHIFT     4
#define TMC5240_HEND_SHIFT      7
#define TMC5240_TBL_SHIFT       15
#define TMC5240_MRES_SHIFT      24
#define TMC5240_INTPOL          (1u << 28)

/* ---- DRV_STATUS (0x6F), p.146 ---- */
#define TMC5240_SG_RESULT_MASK  0x3FFu
#define TMC5240_CS_ACTUAL_SHIFT 16
#define TMC5240_CS_ACTUAL_MASK  0x1Fu
#define TMC5240_S2VSA          (1u << 12)
#define TMC5240_S2VSB          (1u << 13)
#define TMC5240_STEALTH        (1u << 14)
#define TMC5240_FSACTIVE       (1u << 15)
#define TMC5240_STALLGUARD     (1u << 24)
#define TMC5240_OT             (1u << 25)
#define TMC5240_OTPW           (1u << 26)
#define TMC5240_S2GA           (1u << 27)
#define TMC5240_S2GB           (1u << 28)
#define TMC5240_OLA            (1u << 29)
#define TMC5240_OLB            (1u << 30)
#define TMC5240_STST           (1u << 31)

/* ---- SPI_STATUS, returned in bits 39:32 of every datagram (Table 3) ---- */
#define TMC5240_SPI_RESET_FLAG       (1u << 0)
#define TMC5240_SPI_DRIVER_ERROR     (1u << 1)
#define TMC5240_SPI_SG2              (1u << 2)
#define TMC5240_SPI_STANDSTILL       (1u << 3)
#define TMC5240_SPI_VELOCITY_REACHED (1u << 4)
#define TMC5240_SPI_POSITION_REACHED (1u << 5)
#define TMC5240_SPI_STOP_L           (1u << 6)
#define TMC5240_SPI_STOP_R           (1u << 7)
