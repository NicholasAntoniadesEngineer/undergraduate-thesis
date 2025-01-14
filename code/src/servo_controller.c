#include "servo_controller.h"

// Private function prototypes
static bool _is_message_for_this_device(const servo_controller_state_t* state, int received_message);
static int _receive_position(const servo_controller_state_t* state, UART_HandleTypeDef* huart);
static void _update_output(const servo_controller_state_t* state, int position);
static void _set_default_output(const servo_controller_state_t* state);
static int _handle_button_input(const servo_controller_state_t* state, int current_target);
static int _process_pot_value(const servo_controller_state_t* state);
static void _transmit_data(const servo_controller_state_t* state, UART_HandleTypeDef* huart, int target_device, int pot_value);

void servo_controller_init(servo_controller_state_t* state, const servo_controller_config_t* config)
{
    // Copy config values into state
    state->max_devices = config->max_devices;
    state->this_device = config->this_device;
    state->uart_rx_size = config->uart_rx_size;
    state->uart_tx_size = config->uart_tx_size;
    state->uart_timeout = config->uart_timeout;
    state->default_output_pattern = config->default_output_pattern;
    state->uart_delay = config->uart_delay;
    state->rx_delay = config->rx_delay;
    state->output_delay = config->output_delay;
    state->adc_channel = config->adc_channel;
    state->adc_resolution = config->adc_resolution;
    
    // Initialize state values
    state->current_target = 0;
    state->pot_value = 0;
    state->prev_pot_value = 0;

    // Initialize BSP
    stm32_bsp_hal_init();
    stm32_bsp_init_state(&state->bsp_state);

    // Initialize UART with configuration
    stm32_uart_state_t uart_state = {
        .huart = &state->huart1,
        .rx_size = state->uart_rx_size,
        .tx_size = state->uart_tx_size,
        .uart_timeout = state->uart_timeout
    };
    stm32_lib_uart_init(&state->uart_state, &uart_state);

    // Initialize ADC
    stm32_lib_init_adc();
    
    _set_default_output(state);
}

bool servo_controller_process_incoming(servo_controller_state_t* state)
{
    uint8_t received_message;
    if (HAL_UART_Receive(&state->huart1, &received_message, 1, 0) == HAL_OK) 
    {
        if (_is_message_for_this_device(state, received_message)) 
        {
            int position = _receive_position(state, &state->huart1);

            _update_output(state, position);

            return true;
        }
    }
    return false;
}

void servo_controller_process_outgoing(servo_controller_state_t* state)
{
    state->current_target = _handle_button_input(state, state->current_target);
    
    if (state->current_target != 0) 
    {
        state->pot_value = _process_pot_value(state);

        if (state->pot_value != state->prev_pot_value) 
        {
            _transmit_data(state, &state->huart1, state->current_target, state->pot_value);

            state->prev_pot_value = state->pot_value;
        }
    }
}

static bool _is_message_for_this_device(const servo_controller_state_t* state, int received_message)
{
    return received_message == state->this_device;
}

static int _handle_button_input(const servo_controller_state_t* state, int current_target)
{
    if (stm32_lib_check_button_gpioa(0)) 
    {
        stm32_lib_debounce();

        current_target = (current_target % state->max_devices) + 1;

        if (current_target == state->this_device) 
        {
            current_target = (current_target % state->max_devices) + 1;
        }

        BSP_GPIO_WritePin(GPIOB, GPIO_PIN_All, current_target);
        return current_target;
    }
    return current_target;
}

static int _process_pot_value(const servo_controller_state_t* state)
{
    return stm32_lib_adc_input(state->adc_channel, state->adc_resolution);
}

static void _transmit_data(const servo_controller_state_t* state, UART_HandleTypeDef* huart, int target_device, int pot_value)
{
    uint8_t tx_data[2] = {target_device, pot_value};

    BSP_UART_Transmit(huart, tx_data, 2);
    
    stm32_lib_delay(state->uart_delay);
}

static int _receive_position(const servo_controller_state_t* state, UART_HandleTypeDef* huart)
{
    uint8_t position;
    stm32_lib_delay(state->rx_delay);

    BSP_UART_Receive(huart, &position, 1);

    return position;
}

static void _update_output(const servo_controller_state_t* state, int position)
{
    BSP_GPIO_WritePin(GPIOB, GPIO_PIN_All, position);

    stm32_lib_delay(state->output_delay);

    _set_default_output(state);
}

static void _set_default_output(const servo_controller_state_t* state)
{
    BSP_GPIO_WritePin(GPIOB, GPIO_PIN_All, state->default_output_pattern);
} 