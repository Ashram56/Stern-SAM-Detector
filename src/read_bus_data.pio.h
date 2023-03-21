// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------------- //
// read_bus_data //
// ------------- //

#define read_bus_data_wrap_target 0
#define read_bus_data_wrap 3

static const uint16_t read_bus_data_program_instructions[] = {
            //     .wrap_target
    0x34af, //  0: wait   1 pin, 15              [20]
    0x202f, //  1: wait   0 pin, 15                  
    0x4008, //  2: in     pins, 8                    
    0x8820, //  3: push   block                  [8] 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program read_bus_data_program = {
    .instructions = read_bus_data_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config read_bus_data_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + read_bus_data_wrap_target, offset + read_bus_data_wrap);
    return c;
}

// this is a raw helper function for use by the user which sets up the GPIO output, and configures the SM to output on a particular pin
void read_bus_data_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    // all pins are input
    pio_sm_set_consecutive_pindirs(pio, sm, 0, 8, false);
    // configure the state machine
    pio_sm_config c = read_bus_data_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);  // pin 15 is the trigger IOSTB signal
    sm_config_set_in_shift(&c, false, false, 32);
    // initialize and enable the state machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif
