/*
 * stepper -- a small Linux-IIO-style stepper driver subsystem for ESP-IDF
 * and the STM32 HAL.
 *
 * One struct stepper_dev per driver chip. It holds only what every chip
 * here does the same way: the enable pin, the run and hold currents, and a
 * status word. Motion stays with each chip, deliberately. The TMC2209 turns
 * at a velocity from its own pulse generator, or from STEP/DIR pulses the
 * app makes; the TMC5240 runs a ramp to a target position in hardware. One
 * shared move call would be wrong for one of them.
 *
 * Each chip keeps its own handle API (tmc2209.h, tmc5240.h). The handle
 * holds a struct stepper_dev, and tmc2209_stepper() / tmc5240_stepper() hand
 * it out, for code that treats every chip the same way. The handle's init
 * fills it and its del ends it: there is no probe or remove of its own.
 *
 * Every int-returning function returns 0 or a negative errno.
 */
#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * What both chips report, decoded. A flag a chip does not have reads false;
 * each chip's header says which.
 */
struct stepper_status {
	bool standstill;	/* no step for a while: stst */
	bool stealthchop;	/* StealthChop is running right now */
	bool overtemp_warning;	/* otpw: getting hot */
	bool overtemp;		/* ot: shut down until it cools */
	bool short_to_gnd;	/* s2ga/s2gb, either phase: stage disabled */
	bool short_to_supply;	/* s2vsa/s2vsb, either phase: stage disabled */
	bool open_load;		/* ola/olb, either phase; trust it only
				 * turning slowly */
	bool stall;		/* StallGuard has tripped */
	bool was_reset;		/* the chip restarted and lost its settings;
				 * reported once, then cleared */
	uint8_t current_scale;	/* CS_ACTUAL, 0-31: what it really drives */
};

struct stepper_dev;

struct stepper_ops {
	/* drive the enable pin; -EOPNOTSUPP when it is tied */
	int (*enable)(struct stepper_dev *stepper, bool enable);
	/* RMS amperes; hold 0 selects half of run */
	int (*set_current)(struct stepper_dev *stepper, float run_a,
			   float hold_a);
	int (*read_status)(struct stepper_dev *stepper,
			   struct stepper_status *status);
};

struct stepper_dev {
	const char *name;		/* "tmc2209"; log prefix */
	const struct stepper_ops *ops;
	void *priv;			/* the chip's handle */
	uint16_t microsteps;		/* per full step, set by init; read only */
};

/*
 * Turn the output stage on or off with the enable pin. Off, the motor
 * freewheels and holds nothing. -EOPNOTSUPP if the chip's enable is tied.
 */
int stepper_enable(struct stepper_dev *stepper, bool enable);

/*
 * Run and hold current, RMS amperes. hold_a of 0 selects half of run_a. The
 * chip quantises both to 1/32 of its range, so what it drives is near, not
 * equal; current_scale in the status says what it is doing.
 */
int stepper_set_current(struct stepper_dev *stepper, float run_a,
			float hold_a);

/*
 * Read the driver's state into *status, written only on success. Also
 * clears the chip's latched reset flag, so was_reset reports each restart
 * exactly once.
 */
int stepper_read_status(struct stepper_dev *stepper,
			struct stepper_status *status);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */
