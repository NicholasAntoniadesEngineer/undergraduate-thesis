#include <stdint.h>
#include "stm32f0xx.h"
#include "library.h"

#define STM32F051

// Function declarations
static void init_system(void);
static int handle_button_input(int current_slave);
static int process_pot_value(int prev_value);
static void transmit_data(int slave_num, int pot_value);

int main(void) 
{
    int current_slave = 0;
    int pot_value = 0;
    int prev_pot_value = 0; 

    init_system();

    while(1) 
    {
        current_slave = handle_button_input(current_slave);
        pot_value = process_pot_value(prev_pot_value);
        prev_pot_value = pot_value;
        transmit_data(current_slave, pot_value);
    } 
    return 0;
}

static void init_system(void)
{
    libs_init_leds();
    libs_init_switches();
    libs_init_USART1();
}

static int handle_button_input(int current_slave)
{
    if (libs_check_button(0)) 
    {
        libs_write_leds(2);
        return 1;
    } 
    else if (libs_check_button(1)) 
    {
        libs_write_leds(3);
        return 2;
    }
    return current_slave;
}

static int process_pot_value(int prev_value)
{
    int new_value = libs_read_ADC(1,8);
    return new_value;
}

static void transmit_data(int slave_num, int pot_value)
{
    USART1_transmit(slave_num);
    libs_delay(100);
    USART1_transmit(pot_value);
    libs_delay(100);
}

