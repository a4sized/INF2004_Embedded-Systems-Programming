#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#define BUZZER_PIN 22  // GP22

EXPORT void buzzer_init(void);

EXPORT void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms);

EXPORT void buzzer_play_boot_chime(void);

EXPORT void buzzer_play_beep(void);

EXPORT void buzzer_play_alarm(void);

EXPORT void buzzer_samsung_washing_machine(void);

#endif