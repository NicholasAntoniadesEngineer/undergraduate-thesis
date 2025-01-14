#ifndef SERVO_CONTROLLER_H_
#define SERVO_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"
#include "stm32_lib.h"
#include "stm32_bsp.h"

typedef struct {
    uint8_t max_devices;
    uint8_t this_device;
    uint16_t uart_rx_size;
    uint16_t uart_tx_size;
    uint32_t uart_timeout;
    uint8_t default_output_pattern;
    uint32_t uart_delay;
    uint32_t rx_delay;
    uint32_t output_delay;
    uint8_t adc_channel;
    uint8_t adc_resolution;
} servo_controller_config_t;

typedef struct {
    UART_HandleTypeDef huart1;
    bsp_state_t bsp_state;
    stm32_uart_state_t uart_state;
    uint8_t current_target;
    uint8_t pot_value;
    uint8_t prev_pot_value;
    uint8_t max_devices;
    uint8_t this_device;
    uint16_t uart_rx_size;
    uint16_t uart_tx_size;
    uint32_t uart_timeout;
    uint8_t default_output_pattern;
    uint32_t uart_delay;
    uint32_t rx_delay;
    uint32_t output_delay;
    uint8_t adc_channel;
    uint8_t adc_resolution;
} servo_controller_state_t;

// Function declarations
void servo_controller_init(servo_controller_state_t* state, const servo_controller_config_t* config);
bool servo_controller_process_incoming(servo_controller_state_t* state);
void servo_controller_process_outgoing(servo_controller_state_t* state);

#endif /* SERVO_CONTROLLER_H_ */ 