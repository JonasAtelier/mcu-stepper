# Chip reference

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## TMC2209

The cheap, quiet one. StealthChop makes a stepper close to silent; the
single-wire UART sets current and microstepping in software and can move the
motor with no step pulse; StallGuard detects a stall. Datasheet: TMC2209
Rev. 1.08 (2022-05-25), `docs/tmc2209.pdf` (gitignored).

| Function | Does |
|---|---|
| `tmc2209_init(&cfg, &h)` | Take the app's UART, prove the link through IFCNT, apply GCONF, CHOPCONF, TPOWERDOWN and the currents. Leaves the stage off |
| `tmc2209_enable(h, on)` | The output stage, via ENN |
| `tmc2209_set_current(h, run_ma, hold_ma)` | Run and hold current, RMS mA |
| `tmc2209_move(h, usteps_per_s)` / `tmc2209_stop(h)` | Constant velocity from the chip's own generator; 0 hands control back to STEP |
| `tmc2209_read_status(h, &st)` | DRV_STATUS and GSTAT decoded; clears the latched GSTAT flags |
| `tmc2209_stallguard(h, &load)` | SG_RESULT, 0–1023, falling with load |
| `tmc2209_set_stall(h, thr, min_v)` | SGTHRS and TCOOLTHRS: DIAG goes high on a stall above min_v |
| `tmc2209_read` / `tmc2209_write` | Raw register access |
| `tmc2209_del(h)` | Stage off, free. The UART stays the app's |

### Datagrams and the echo

Section 4.1 (p.18-19): a write is eight bytes — sync nibble `1010` sent LSB
first (0x05 on the wire), slave address, register with bit 7 set, four data
bytes high to low, CRC. A read request is four — sync, address, register,
CRC — and the chip answers after SENDDELAY with eight, addressed to master
0xFF. The CRC is CRC8-ATM, polynomial 0x07, init 0, applied LSB to MSB
(4.2, p.20, which gives the loop in C); `test/test_tmc2209_math.c` checks it
against the textbook formulation (reflect each byte, then MSB-first CRC-8).

On the usual single wire the MCU hears its own datagram before any reply.
The driver sends every datagram through the bus layer's `exchange()`, which
collects as many bytes as went out, and drops them; then `receive()` takes
the eight-byte reply. A reply must come from 0xFF, name the register asked
for and pass its CRC, or the read is `-EBADMSG`. The echo is not compared
with what was sent.

There is no ID register. Init reads IFCNT, writes SLAVECONF (SENDDELAY 2,
three times eight bit times, which the datasheet asks for on a shared bus,
4.1.2 p.19) and reads IFCNT again: one more successful write proves both
directions and the CRC. The counter wraps at 255.

### Current

Section 9 (p.53):

```
I_RMS = (CS + 1) / 32 * VFS / (RSENSE + 20 mOhm) / sqrt(2)
```

VFS is 325 mV with `vsense` clear and 180 mV with it set (VSRTL, VSRTH;
20.2, p.78). The driver tries the high range, and drops to the low one for
twice the resolution when the run current is under half its scale. Run and
hold share the one `vsense` bit, so the hold CS is moved onto the run
current's range. IHOLDDELAY is 10, so the current fades to hold rather than
dropping in one step. `GCONF.I_scale_analog` is cleared: the VREF trimmer no
longer scales anything.

### Velocity, StallGuard

`tmc2209_move()` writes VACTUAL, 24-bit two's complement, in units of
fCLK / 2^24 (p.67; ±(2^23)−1, 5.2 p.28). `clock_hz` 0 means the internal
12 MHz oscillator, which is not trimmed: velocities can be several per cent
out with nothing reporting it.

TCOOLTHRS is a step period in clock ticks between two 1/256 microsteps,
whatever MRES is (5.2, p.28), so the driver converts
`fCLK / (v * 2^MRES)`, clamped to its 20 bits. StallGuard is meaningless at
low speed; the floor is not optional. SG_RESULT is ten bits (5.3, p.29).

### Registers

Every address and field in `src/tmc2209/tmc2209_regs.h` names its section
and page: GCONF, GSTAT, IFCNT, SLAVECONF (5.1, p.23-24), IHOLD_IRUN,
TPOWERDOWN, TSTEP, TPWMTHRS, VACTUAL (5.2, p.28), TCOOLTHRS, SGTHRS,
SG_RESULT (5.3, p.29), CHOPCONF (5.5.1, p.33-34), DRV_STATUS (5.5.3, p.37).
esp-tmc2209-stepper cited DRV_STATUS as 5.5.2, which is PWMCONF.

## TMC5240

The TMC5240 contains the ramp generator: give it a target position and it
accelerates, cruises and decelerates in hardware. There is no step pulse
anywhere in this driver. SPI only; the chip's UART is not implemented.
Datasheet: `docs/tmc5240.pdf` (gitignored).

| Function | Does |
|---|---|
| `tmc5240_init(&cfg, &h)` | Probe INP_OUT.VERSION, ramp down a motor still turning from before a reset, set current, chopper, microsteps and a default ramp, enable the stage |
| `tmc5240_set_ramp(h, &r)` | Velocities and accelerations in microsteps/s, converted to register units; all fields checked before any is written |
| `tmc5240_move_to` / `tmc5240_move_by` | Absolute or relative target; returns at once |
| `tmc5240_run(h, v)` | Constant velocity, sign selects direction |
| `tmc5240_stop` / `tmc5240_stop_hard` | Ramp down, or stop as fast as AMAX allows |
| `tmc5240_wait(h, ms)` | Block until position reached |
| `tmc5240_position` / `tmc5240_velocity` / `tmc5240_set_position` | XACTUAL, VACTUAL, and redefining the position at standstill |
| `tmc5240_status(h, &st)` | Ramp flags, position, velocity and all of DRV_STATUS, in one read chain |
| `tmc5240_set_current` / `tmc5240_run_current` | Change current; what CS actually delivers |
| `tmc5240_supply_voltage` / `tmc5240_temperature` | The on-chip ADC |
| `tmc5240_stallguard(h, thr, vmin)` | StallGuard4 threshold and its velocity window |
| `tmc5240_read` / `tmc5240_write` / `tmc5240_spi_status` | Raw access, and SPI_STATUS from the last datagram |

### One read is two datagrams

Every access is 40 bits, address byte then four data bytes, MSB first, SPI
mode 3 (p.24-25). A datagram returns the data for the address sent in the
*previous* one, so n registers take n + 1 datagrams; `tmc5240_status()` and
`stepper_read_status()` chain theirs. The eight bits shifted back over the
address byte are SPI_STATUS (Table 3). The bus layer's `exchange()` carries
each datagram, under its own chip select.

### Acceleration scale

Table 19, "Ramp Generator Parameters vs. Units" (p.49):

```
v[Hz]   = v[TMC5240] * fCLK / 2^24
a[Hz/s] = a[TMC5240] * fCLK^2 / 2^42
```

esp-tmc5240-stepper divided by 2^41, citing the same table. 2^41 is the
TMC5130/5160 datasheets' figure — their `(512 * 256) * 2^24` form. So the
factor of two is either a real change in the TMC5240 or an error in its
datasheet; nothing here can tell which. **This library follows the TMC5240
datasheet**, and every acceleration register now holds twice what the
original wrote.

It wants confirming on hardware. On a free-running motor with a known
external clock (or the internal one measured), set `vstart = 0`, a VMAX
`v` and an AMAX `a` with `v1 = v2 = 0`, run from standstill, and time how
long it takes until RAMP_STAT.velocity_reached sets (or VACTUAL first reads
`v`). That time is `v / a` with 2^42, or `v / (2a)` if the silicon is on
2^41. Seconds apart, at for example `v = 51200`, `a = 25600` microsteps/s.
If it is 2^41, change `TMC5240_TWO_P42` in `src/tmc5240/tmc5240_math.h`
back and the two lines of `test/test_tmc5240_math.c` with it.

### The clock is the calibration knob

Every velocity and acceleration register is counted in units of fCLK. The
chip does not know its own clock frequency, and neither does the driver
unless told. Leave `clock_hz` at 0 and the internal oscillator is assumed to
be its 12.5 MHz nominal (IOIN.EXT_CLK, p.95) — but the datasheet specifies
11.9 to 13.2 MHz, so a motor can run 5% off the speed asked for, with no
error anywhere. Drive CLK from the MCU and put the frequency in `clock_hz`
when speed has to be right. The driver warns when `clock_hz` disagrees with
INP_OUT.EXT_CLK.

"Hz" throughout means **microsteps per second** at the sequencer's 1/256
resolution. One revolution of a 200-fullstep motor at 256 microsteps is
51200 of them.

### Motor current

Full-scale current comes from the resistor on IREF (p.45-46):

```
IFS(peak) = KIFS / RREF(kOhm)      KIFS = 11.75, 24 or 36 by DRV_CONF.CURRENT_RANGE (Table 16)
I(RMS)    = IFS / sqrt(2) * (CS + 1) / 32
```

`rref_kohm` must be the resistor fitted, 12 to 60 kOhm. The driver picks the
lowest range that reaches the run current ("select the lowest fitting
range", p.98), leaves GLOBAL_SCALER at full scale, and warns if CS
quantisation misses by more than 5%. Table 17's matrix is checked in
`test/test_tmc5240_math.c`.

### Registers

Every address in `src/tmc5240/tmc5240_regs.h` is cited to Table 27,
"Overview of Register Map" (p.81-83), and to its register entry's page; bit
fields come from that entry. esp-tmc5240-stepper took the addresses from
ADI's TMC-API; all of them match the datasheet except AACTUAL (0x31), which
the datasheet does not list and the driver never used, so it is gone. The
ramp field widths are VMAX 23 bits, V1/V2 20, VSTART/VSTOP 18 and every
acceleration 18 (p.105-115) — wider than a TMC5160's.

## Provenance

- Both drivers are ports of esp-tmc2209-stepper (92f291b) and
  esp-tmc5240-stepper (fe3530d); their logic is unchanged apart from what
  [API usage](usage.md#changes-from-esp-tmc2209-stepper) lists.
- Every register value is quoted from the datasheets in `docs/` with its
  section, table or page.
- Nothing has been checked against a physical chip.
