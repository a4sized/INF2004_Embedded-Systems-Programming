#ifndef __size_t_defined
#define __size_t_defined
#endif

#include <stdint.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "../app.h"
#include "buzzer.h"

#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_E6  1319
#define REST     0

typedef struct {
    uint16_t frequency; // Hz
    uint16_t duration;  // ms
} Note;

LOCAL const Note samsung_melody[] = {
    {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_G5, 300}, {NOTE_G5, 300},
    {NOTE_A5, 150}, {NOTE_G5, 150}, {NOTE_E5, 300}, {NOTE_C5, 300},
    {NOTE_D5, 150}, {NOTE_E5, 150}, {NOTE_F5, 300}, {NOTE_D5, 300},
    {NOTE_E5, 300}, {NOTE_C5, 300}, {REST,    150},
    {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_G5, 300}, {NOTE_C6, 300},
    {NOTE_D6, 150}, {NOTE_C6, 150}, {NOTE_B5, 300}, {NOTE_G5, 300},
    {NOTE_A5, 150}, {NOTE_B5, 150}, {NOTE_C6, 300}, {NOTE_E5, 300},
    {NOTE_D5, 300}, {NOTE_C5, 450}, {REST,    200}
};

EXPORT void buzzer_init(void) {
    gpio_set_pin(BUZZER_PIN, GPIO_MODE_OUT);
    gpio_set_val(BUZZER_PIN, 0);
}

EXPORT void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0) {
        tk_dly_tsk(duration_ms);
        return;
    }

    // Set GP22 as standard digital output
    gpio_set_pin(BUZZER_PIN, GPIO_MODE_OUT);

    // Calculate half-period in microseconds
    // Period = 1,000,000 / frequency_hz
    uint32_t half_period_us = 500000UL / frequency_hz;
    uint32_t total_cycles = (frequency_hz * duration_ms) / 1000UL;

    for (uint32_t i = 0; i < total_cycles; i++) {
        gpio_set_val(BUZZER_PIN, 1);
        
        // Busy-wait delay for half period
        for (volatile uint32_t d = 0; d < half_period_us * 3; d++) {
            __asm__("nop");
        }

        gpio_set_val(BUZZER_PIN, 0);

        // Busy-wait delay for half period
        for (volatile uint32_t d = 0; d < half_period_us * 3; d++) {
            __asm__("nop");
        }
    }

    gpio_set_val(BUZZER_PIN, 0);
}

/*EXPORT void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0) {
        tk_dly_tsk(duration_ms);
        return;
    }

     
    pwm_set_pin(BUZZER_PIN);

  
     
    UW wrap = 1000000U / frequency_hz; 
    pwm_set_wrap(BUZZER_PIN, wrap);
    pwm_set_cc(BUZZER_PIN, wrap / 2);   

    pwm_set_enabled(BUZZER_PIN, TRUE);

    tk_dly_tsk(duration_ms);

    pwm_set_enabled(BUZZER_PIN, FALSE);
    gpio_set_pin(BUZZER_PIN, GPIO_MODE_OUT);
    gpio_set_val(BUZZER_PIN, 0);
}*/

EXPORT void buzzer_play_boot_chime(void) {
    buzzer_play_tone(523, 80);  // C5
    tk_dly_tsk(20);
    buzzer_play_tone(659, 80);  // E5
    tk_dly_tsk(20);
    buzzer_play_tone(784, 120); // G5
}

EXPORT void buzzer_play_beep(void) {
    buzzer_play_tone(2000, 60); // Short beep
}

EXPORT void buzzer_play_alarm(void) {
    for (SZ i = 0; i < 3; i++) {
        buzzer_play_tone(3000, 60); 
        tk_dly_tsk(30);
        buzzer_play_tone(1500, 60); 
        tk_dly_tsk(30);
    }
}

EXPORT void buzzer_samsung_washing_machine(void) {
    SZ num_notes = sizeof(samsung_melody) / sizeof(samsung_melody[0]);

    for (SZ i = 0; i < num_notes; i++) {
        uint16_t freq = samsung_melody[i].frequency;
        uint16_t duration = samsung_melody[i].duration;

        if (freq == REST) {
            tk_dly_tsk(duration);
        } else {
            uint16_t tone_dur = (duration > 15) ? (duration - 15) : duration;
            buzzer_play_tone(freq, tone_dur);
            tk_dly_tsk(15);
        }
    }
}