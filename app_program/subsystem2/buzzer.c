#include "pico/stdlib.h"
#include "hardware/pwm.h"

#ifndef __size_t_defined
#define __size_t_defined
#endif

#include <tk/tkernel.h>
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
    uint16_t frequency; // hz
    uint16_t duration;  // ms
} Note;

static const Note samsung_melody[] = {
    
    {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_G5, 300}, {NOTE_G5, 300},
    {NOTE_A5, 150}, {NOTE_G5, 150}, {NOTE_E5, 300}, {NOTE_C5, 300},
    {NOTE_D5, 150}, {NOTE_E5, 150}, {NOTE_F5, 300}, {NOTE_D5, 300},
    {NOTE_E5, 300}, {NOTE_C5, 300}, {REST,    150},
    {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_G5, 300}, {NOTE_C6, 300},
    {NOTE_D6, 150}, {NOTE_C6, 150}, {NOTE_B5, 300}, {NOTE_G5, 300},
    {NOTE_A5, 150}, {NOTE_B5, 150}, {NOTE_C6, 300}, {NOTE_E5, 300},
    {NOTE_D5, 300}, {NOTE_C5, 450}, {REST,    200}
};

void buzzer_init(void){
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);
}

void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms){

    if (frequency_hz == 0){
        tk_dly_tsk(duration_ms);
        return;
    }

    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    uint32_t clock_freq = 125000000U;
    uint32_t divider = 125U; 
    uint32_t wrap = clock_freq / (divider * frequency_hz) - 1U;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)divider);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(BUZZER_PIN), wrap / 2U);

    tk_dly_tsk(duration_ms);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0U);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);
}








void buzzer_play_boot_chime(void){
    buzzer_play_tone(523, 80);  // C5
    tk_dly_tsk(20);
    buzzer_play_tone(659, 80);  // E5
    tk_dly_tsk(20);
    buzzer_play_tone(784, 120); // G5
}

void buzzer_play_beep(void){
    buzzer_play_tone(2000, 60); // short beep
}

void buzzer_play_alarm(void){ // can be used for faults
    for (int i = 0; i < 3; i++){
        buzzer_play_tone(3000, 60); 
        tk_dly_tsk(30);
        buzzer_play_tone(1500, 60); 
        tk_dly_tsk(30);
    }
}

void buzzer_samsung_washing_machine(void){
    size_t num_notes = sizeof(samsung_melody) / sizeof(samsung_melody[0]);

    for (size_t i = 0; i < num_notes; i++){
        uint16_t freq = samsung_melody[i].frequency;
        uint16_t duration = samsung_melody[i].duration;

        if (freq == REST){
            tk_dly_tsk(duration);
        } else{
            uint16_t tone_dur = (duration > 15) ? (duration - 15) : duration;
            buzzer_play_tone(freq, tone_dur);
            tk_dly_tsk(15);
        }
    }
}