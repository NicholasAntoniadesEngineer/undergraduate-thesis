#include <stdint.h>
#include "stm32f0xx.h"
#include "library.h"

#define STM32F051


int main(void) {
    // initialisations here
	libs_init_leds();
	libs_init_switches();
	libs_init_USART1();
    int slave_num;
    int potvalue;
	
	
    // hang in an infinite loop
    while(1) {

    	//Receive
        

		message = USART1_receive();
        if(message == 1){           //adjust this to the correct slave number
		    libs_delay(30);
            position = USART1_receive();    
        }
		libs_delay(30);

        } 
	
    	
    	//Receive
		GPIOB ->ODR = USART1_receive();
		libs_delay(200);
    	GPIOB ->ODR = 0b111;

    }
    return 0;  // keep compiler happy with a return code.

}

