#include <stdint.h>
#include "stm32f0xx.h"
#include "library.h"

#define STM32F051
#define SLAVE_NUMBER 1 

static void init_system(void);
static bool is_message_for_this_slave(int received_message);
static int receive_position(void);
static void update_output(int position);
static void set_default_output(void);

int main(void) 
{
    init_system();
    set_default_output();

    while(1) 
    {
        int received_message = USART1_receive();
        
        if(is_message_for_this_slave(received_message)) 
        {
            int position = receive_position();
            update_output(position);
        }
    }
    return 0;
}

static void init_system(void)
{
    libs_init_leds();
    libs_init_switches();
    libs_init_USART1();
    // Initialize GPIOB for output
    // ... (add any necessary GPIO initialization)
}

static bool is_message_for_this_slave(int received_message)
{
    return received_message == SLAVE_NUMBER;
}

static int receive_position(void)
{
    libs_delay(30);  // Small delay to ensure data is ready
    return USART1_receive();
}

static void update_output(int position)
{
    GPIOB->ODR = position;
    libs_delay(200);  // Hold the position briefly
    set_default_output();
}

static void set_default_output(void)
{
    GPIOB->ODR = 0b111;  // Default state when not actively showing position
}

