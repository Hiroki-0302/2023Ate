//compiled in https://wokwi.com/tools/pioasm
// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------------------ //
// pio_rotary_encoder //
// ------------------ //

#define pio_rotary_encoder_wrap_target 0
#define pio_rotary_encoder_wrap 23

static const uint16_t pio_rotary_encoder_program_instructions[] = {
            //     .wrap_target
    0x0011, //  0: jmp    17                         
    0x0015, //  1: jmp    21                         
    0x0017, //  2: jmp    23                         
    0x0011, //  3: jmp    17                         
    0x0017, //  4: jmp    23                         
    0x0011, //  5: jmp    17                         
    0x0011, //  6: jmp    17                         
    0x0015, //  7: jmp    21                         
    0x0015, //  8: jmp    21                         
    0x0011, //  9: jmp    17                         
    0x0011, // 10: jmp    17                         
    0x0017, // 11: jmp    23                         
    0x0011, // 12: jmp    17                         
    0x0017, // 13: jmp    23                         
    0x0015, // 14: jmp    21                         
    0x0011, // 15: jmp    17                         
    0x4002, // 16: in     pins, 2                    
    0xa0e6, // 17: mov    osr, isr                   
    0x60c2, // 18: out    isr, 2                     
    0x4002, // 19: in     pins, 2                    
    0xa086, // 20: mov    exec, isr                  
    0xc000, // 21: irq    nowait 0                   
    0x0011, // 22: jmp    17                         
    0xc001, // 23: irq    nowait 1                   
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program pio_rotary_encoder_program = {
    .instructions = pio_rotary_encoder_program_instructions,
    .length = 24,
    .origin = 0,
};

static inline pio_sm_config pio_rotary_encoder_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + pio_rotary_encoder_wrap_target, offset + pio_rotary_encoder_wrap);
    return c;
}
#endif
