#include <stdint.h>
#include "stm32f0xx.h"
#include "library.h"

#define STM32F051

// Function declarations
static void init_system(void);
static int receive_message(void);
static int receive_position(int message);
static void update_output(int position);

int main(void) 
{
    init_system();
    int current_message = 0;
    int current_position = 0;

    while(1) 
    {
        current_message = receive_message();
        if(current_message == 1) {  // adjust this to the correct slave number
            current_position = receive_position(current_message);
            update_output(current_position);
        }
    }
    return 0;
}

static void init_system(void)
{
    libs_init_leds();
    libs_init_switches();
    libs_init_USART1();
}

static int receive_message(void)
{
    return USART1_receive();
}

static int receive_position(int message)
{
    libs_delay(30);
    return USART1_receive();
}

static void update_output(int position)
{
    GPIOB->ODR = position;
    libs_delay(200);
    GPIOB->ODR = 0b111;
}

