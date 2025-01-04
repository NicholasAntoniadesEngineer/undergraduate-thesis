#ifndef library
#define library

#include <stdint.h>
#define STM32F051
#include "stm32f0xx.h"

//Prototypes

//Initialisations
void libs_init_leds(void);
void libs_init_switches(void);
void libs_init_ADC_8bit(void);
void libs_init_TIM6(uint32_t arr, uint32_t psc);
void libs_TIM6_PSC(uint32_t psc);
void libs_TIM6_ARR(uint32_t arr);
void libs_init_USART(void);
void init_PWM(void);

//Check GPIOs
int libs_check_button(int button);
int libs_check_button_press(int button);
int libs_check_button_release(int button) ;
void libs_debounce(void);
void libs_ack_irq(void);

//functions
int libs_read_ADC(int pot, int resolution);
void libs_sort_into_another_array(uint8_t *param, uint8_t *target, int size);
void libs_write_leds(uint8_t);
uint8_t libs_read_leds(void);
void libs_delay(int time);

#endif // LIBS_HEADER_INCLUDED
