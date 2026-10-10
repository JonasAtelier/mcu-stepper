/*
 * Pure TMC2209 conversions, split out so they can be compiled and asserted
 * on the host (see test/test_tmc2209_math.c). Every constant is quoted from
 * docs/tmc2209.pdf (Rev. 1.08), with its section and page.
 */
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Full-scale sense voltage, 20.2 p.78: VSRTL 325 mV with vsense clear,
 * VSRTH 180 mV with it set. Millivolts, to stay in integers where it
 * matters.
 */
#define TMC2209_VFS_HIGH_MV     325
#define TMC2209_VFS_LOW_MV      180

/* VACTUAL counts in units of fCLK / 2^24, p.67; 24 bits signed, so
 * +-(2^23)-1, 5.2 p.28. */
#define TMC2209_VACTUAL_SCALE   16777216.0f
#define TMC2209_VACTUAL_MAX     8388607

/* TCOOLTHRS is 20 bits, 5.3 p.29. */
#define TMC2209_TCOOLTHRS_MAX   0x000FFFFFu

/*
 * CRC8-ATM, polynomial 0x07, initial value zero, applied LSB to MSB over every
 * byte of the datagram except the CRC itself. 4.2 p.20, which gives this
 * same loop in C.
 */
static inline uint8_t tmc2209_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    size_t i;
    int bit;

    for (i = 0; i < len; i++) {
        uint8_t byte = data[i];

        for (bit = 0; bit < 8; bit++) {
            if ((crc >> 7) ^ (byte & 0x01))
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);

            byte >>= 1;
        }
    }

    return crc;
}

/*
 * Section 9, p.53:
 *
 *   I_RMS = (CS + 1) / 32  *  VFS / (Rsense + 20 mOhm)  *  1 / sqrt(2)
 *
 * turned round to give CS. vsense picks VFS: the low range gives finer steps
 * at small currents, so use it whenever the current fits.
 */
static inline uint8_t tmc2209_current_to_cs(float rsense_mohm, uint16_t ma,
                                            bool *vsense)
{
    float denom_mohm = rsense_mohm + 20.0f;
    float cs;
    int rounded;

    /* Try the high range first; it is the one that reaches full current. */
    cs = 32.0f * 1.41421356f * (float)ma * denom_mohm /
         (1000.0f * (float)TMC2209_VFS_HIGH_MV) - 1.0f;
    *vsense = false;

    /* Under half scale, the low range gives twice the resolution for free. */
    if (cs < 15.0f) {
        cs = 32.0f * 1.41421356f * (float)ma * denom_mohm /
             (1000.0f * (float)TMC2209_VFS_LOW_MV) - 1.0f;
        *vsense = true;
    }

    rounded = (int)lroundf(cs);
    if (rounded < 0)
        rounded = 0;
    if (rounded > 31)
        rounded = 31;

    return (uint8_t)rounded;
}

/* What that CS actually delivers, so the log can say what you got rather than
 * what you asked for. */
static inline uint16_t tmc2209_cs_to_current(float rsense_mohm, uint8_t cs,
                                             bool vsense)
{
    float vfs_mv = vsense ? (float)TMC2209_VFS_LOW_MV
                          : (float)TMC2209_VFS_HIGH_MV;

    return (uint16_t)(((float)cs + 1.0f) / 32.0f * vfs_mv * 1000.0f /
                      ((rsense_mohm + 20.0f) * 1.41421356f));
}

/* Both currents share one vsense bit, so a CS worked out on cs_vsense's
 * range has to be scaled onto vsense's, keeping the current it gives. */
static inline uint8_t tmc2209_cs_on_range(uint8_t cs, bool cs_vsense,
                                          bool vsense)
{
    float vfs_ratio;
    int scaled;

    if (cs_vsense == vsense)
        return cs;
    vfs_ratio = vsense ? (float)TMC2209_VFS_HIGH_MV / (float)TMC2209_VFS_LOW_MV
                       : (float)TMC2209_VFS_LOW_MV / (float)TMC2209_VFS_HIGH_MV;
    scaled = (int)lroundf(((float)cs + 1.0f) * vfs_ratio) - 1;

    return (uint8_t)(scaled < 0 ? 0 : (scaled > 31 ? 31 : scaled));
}

/* CHOPCONF.MRES, 5.5.1 p.33: 0 is native 256; every step down doubles the
 * step angle, up to 8 for full steps. -1 for anything else. */
static inline int tmc2209_mres(uint16_t microsteps)
{
    switch (microsteps) {
    case 256: return 0;
    case 128: return 1;
    case 64:  return 2;
    case 32:  return 3;
    case 16:  return 4;
    case 8:   return 5;
    case 4:   return 6;
    case 2:   return 7;
    case 1:   return 8;
    default:  return -1;
    }
}

/* VACTUAL for a velocity in microsteps per second at clock_hz. false when
 * it does not fit the register's +-(2^23)-1. */
static inline bool tmc2209_vactual(float usteps_per_s, uint32_t clock_hz,
                                   int32_t *vactual)
{
    float ticks = usteps_per_s * TMC2209_VACTUAL_SCALE / (float)clock_hz;

    if (ticks > TMC2209_VACTUAL_MAX || ticks < -TMC2209_VACTUAL_MAX)
        return false;

    *vactual = (int32_t)lroundf(ticks);
    return true;
}

/* TCOOLTHRS is a step *period*, not a speed: clock ticks between two
 * 1/256 microsteps, which the datasheet is explicit stays 1/256 whatever
 * MRES is (5.2 p.28). So convert the caller's microsteps per second into
 * 1/256 ones first. Faster motion means a smaller number, so the comparison
 * runs the other way round from what you would expect. */
static inline uint32_t tmc2209_tcoolthrs(uint32_t clock_hz,
                                         float min_usteps_per_s, uint8_t mres)
{
    uint32_t tcoolthrs = (uint32_t)((float)clock_hz /
                                    (min_usteps_per_s *
                                     (float)(1u << mres)));

    if (tcoolthrs > TMC2209_TCOOLTHRS_MAX)
        tcoolthrs = TMC2209_TCOOLTHRS_MAX;
    return tcoolthrs;
}
