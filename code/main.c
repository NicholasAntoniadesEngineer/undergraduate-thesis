/******************************************************************************
 * @file    main.c
 * @brief   Main program for servo controller system
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"
#include "servo_controller.h"

/**
 * @brief Process incoming and outgoing messages
 * @param state Pointer to servo controller state structure
 */
static void _process_messages(servo_controller_state_t* state) 
{
    /* Process incoming messages */
    servo_controller_process_incoming(state);
    
    /* Process outgoing messages */
    servo_controller_process_outgoing(state);
}

/**
 * @brief  Main function
 * @retval int
 */
int main(void) 
{
    servo_controller_state_t controller_state = {0};
    
    servo_controller_config_t config = {
        .max_devices = 4,
        .this_device = 1,
        .uart_rx_size = 64,
        .uart_tx_size = 64,
        .uart_timeout = 1000,
        .default_output_pattern = 0x00,
        .uart_delay = 10,
        .rx_delay = 5,
        .output_delay = 20,
        .adc_channel = 0,
        .adc_resolution = 8
    };

    servo_controller_init(&controller_state, &config);

    while(1) 
    {
        _process_messages(&controller_state);
    }
    
    return 0;
}
