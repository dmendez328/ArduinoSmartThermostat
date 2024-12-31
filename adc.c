#include <avr/io.h>

#include "adc.h"

# define maskbits1 0x0f // This is the maskbits for doing several things

void adc_init(void) {
    // Initialize the ADC
    // Initializing ADMUX
    ADMUX &= ~(1 << REFS1);
    ADMUX |= (1 << REFS0);
    ADMUX |= (1 << ADLAR);
    ADMUX &= ~(1 << 4);
    ADMUX &= ~(1 << MUX3);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX0);
    // Initializing ADCSRA
    ADCSRA |= (1 << ADPS2); //Prescalar 
    ADCSRA |= (1 << ADPS1); //Prescalar 
    ADCSRA |= (1 << ADPS0); //Prescalar 
    ADCSRA |= (1 << ADEN); //Enables bit to act as an on & off switch
    ADCSRA &= ~(1 << ADSC);
    ADCSRA &= ~(1 << ADATE);
    ADCSRA &= ~(1 << ADIF);
    ADCSRA &= ~(1 << ADIE);
}

uint8_t adc_sample(uint8_t channel) {
    // Set ADC input mux bits to 'channel' value
    ADMUX &= ~(maskbits1);
    ADMUX |= (channel & maskbits1);

    // Convert an analog input and return the 8-bit result
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & 0b01000000) != 0) {

    }

    unsigned char result = ADCH;

}
