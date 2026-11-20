#pragma once
// OPTIONAL MCP23017 -> eight relay inputs (channels 0..7) bench tests ONLY.
// IMPORTANT: This is NOT an eight-GPIO, directly wired relay-board driver.
// Default 0: no MCP23017 I2C access; standard range test is unchanged.
#define BEL_ENABLE_OPTIONAL_RELAY_BENCH 0

// MCP23017 I2C address, commonly 0x20 with A0/A1/A2 grounded.
#define BEL_RELAY_I2C_ADDRESS 0x20

// Choose the electrical polarity when enabling relay bench tests:
//   0: HIGH energizes relay input; LOW turns it off.
//   1: LOW energizes relay input; HIGH turns it off.
// Leave -1 until you verify your particular module. Never guess.
#define BEL_RELAY_ACTIVE_LOW -1

#if BEL_ENABLE_OPTIONAL_RELAY_BENCH && (BEL_RELAY_ACTIVE_LOW != 0 && BEL_RELAY_ACTIVE_LOW != 1)
#error "Set BEL_RELAY_ACTIVE_LOW to 0 or 1 after confirming the relay module's active polarity."
#endif

// Manual USB-serial single-channel pulse: nominal 100 ms (software-timed;
// blocking tasks may extend it). Use NO loads / solenoids / igniters.
#define BEL_RELAY_BENCH_PULSE_MS 100UL
