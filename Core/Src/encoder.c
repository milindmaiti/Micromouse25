#include "encoder.h"
#include "util.h"
#include "stm32f2xx_hal.h"
#include <stdint.h>

volatile int Rencoder_count;
volatile int Lencoder_count;

void encoder_reset() {
	Rencoder_count = 0;
	Lencoder_count = 0;
}

void update_encoder(uint32_t pin){
	if(HAL_GPIO_ReadPin(GPIOA, pin)){
		if(pin == Renca_pin)
		Rencoder_count++;
		else
		Lencoder_count++;
	}
	else{
		if(pin == Renca_pin)
		Rencoder_count--;
		else
		Lencoder_count--;
	}
}


int get_encoder_right(){
	return Rencoder_count;
}
int get_encoder_left(){
	return Lencoder_count;
}


