#include "encoder.h"

volatile int Rencoder_count;
volatile int Lencoder_count;

void encoder_reset() {
	Rencoder_count = 0;
	Lencoder_count = 0;
}

void update_encoder(uint32_t pin){
	if(HAL_GPIO_READ(pin)){
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


