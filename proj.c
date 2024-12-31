/********************************************
 *
 *  Name: Diego Mendez
 *  Email: diegomen@usc.edu
 *  Section: 31291
 *  Assignment: Project - Thermostat
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "ds18b20.h"
#include "lcd.h"

volatile unsigned char rot_check;


unsigned int celsius; // Integer that stores the celsius value
unsigned char f; // This is the stored farenheit


unsigned char button_check; // For Pin Change Interrupts on Group D
volatile char button_low; // Low Button value stored - 0 or 1
volatile char button_high; // High Button value stored - 0 or 1
volatile int button_change = 0; // Button for temp buttons, flag 


volatile int high_count;  // Initial value of the high temp
volatile int low_count;  // Initial value of the low temp
volatile int current_temp; // This is the current temperature that is used as a reference for many things (i.e. LED Light Check)


void timer0_init(void); // Timer 0 init function
volatile int check_buzzer_played = 0; // Serves as a boolean to see if the buzzer was played
volatile int val = 0; // Value that the counter has to reach in order for the buzzer to stop
volatile count2 = 0; // Counter to see how many times the ISR was run for Buzzer


void timer1_init(void); // Timer 1 init function


void timer2_init(void); // Timer 2 init function


volatile int button_changed_recent = 0; // Serves as a flag to see if a button was pressed recently to point at the temp of the high and low counters

void play_note(uint16_t); // Play note for buzzer function


// Frequencies for natural notes from middle C (C4)cup one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };


// All the following variables need to be volatile because they are used in the ISR and the mainprogram

volatile char new_state, old_state; // States for buttons: was low pressed or high pressed?
volatile char new_rot_state, old_rot_state; // Rotary encoder states, used for changing and incrementing values of high and low
volatile char changed = 0;  // Flag for incrementing the high and low values and displaying them
volatile char a, b; // Characters that help observe the state of the rotary encoder and how it will change


unsigned char temp_data[2]; // Unsigned array as an argument for ds_temp()

int main(void) {

	high_count = eeprom_read_byte((void *) 0); // Reads values from the EEPROM Memory for high
	low_count = eeprom_read_byte((void *) 100); // Reads values from the EEPROM Memory for low


	// Initialize the LCD, Temperature Sensor
	lcd_init();
	ds_init();


    // Initialize DDR and PORT registers and LCD and Rotary Encoder inputs


	PORTC |= (1 << PC1) | (1 << PC2); // For the inputs going into the bits from the rotary encoder


	sei(); // Enable global interrupts


    // Write a spash screen to the LCD
    lcd_writecommand(1);
    lcd_moveto(0,0);
    lcd_stringout("EE109 Project");
    lcd_moveto(1,0);
    lcd_stringout("Diego Mendez");
    _delay_ms(1000);
    lcd_writecommand(1);


	/* Setting up the screen display */

	// Row 1
	lcd_moveto(0,0);
	lcd_stringout("Temp: ");

	// Row 2

	// Low counter set-up
	lcd_moveto(1,0);
	lcd_stringout("Low= ");
	
	char disp_begin_low[3];
	snprintf(disp_begin_low, 3, "%2d", low_count);
	lcd_stringout(disp_begin_low);
	

	// High counter set-up
	lcd_moveto(1,7);
	lcd_stringout(" High= ");

	char disp_begin_high[3];
	snprintf(disp_begin_high, 3, "%2d", high_count);
	lcd_stringout(disp_begin_high);


	/* Beginning Register Bits for the LED */
	
	DDRC |= ((1 << 3) | (1 << 4) | (1 << 5));

	PORTC &= ~(1 << 4);
	PORTC |= ((1 << 3) | (1 << 5));


	
	/* Buzzer Register bit(s) */

	DDRB |= (1 << 5);


	/* Buttons Initial Register Bits */

	PORTD |= ((1 << 3) | (1 << 2));
	PCICR |= (1 << 2);
	PCMSK2 |= ((1 << 3) | (1 << 2));


	/* Servo Motor */

	DDRB |= (1 << 3);


	/* Rotary Encoder Reading and Set-up */

	PORTC |= ((1 << 1) | (1 << 2));

	// Following sets up ISR reading, for pin interrupt(s)
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT9);
	PCMSK1 |= (1 << PCINT10);


	rot_check = PINC; // Read all of PIN C

	// Store the values (either 1 or 0) as characters
	a = (rot_check & (1 << PC1)) >> 1; 
	b = (rot_check & (1 << PC2)) >> 2;  

	// Compare the values of the characters to determine the initial state
    if (!b && !a)
	old_rot_state = 0;
    else if (!b && a)
	old_rot_state = 1;
    else if (b && !a)
	old_rot_state = 2;
    else
	old_rot_state = 3;
	
	/* Starts off the state machine for the rotary encoder */
	new_rot_state = old_rot_state;


	/* This state starts off the state for the buttons */
	old_state = 0;
    new_state = old_state;


	// Always start with a ds_convert() to get ready for a ds_temp()
	ds_convert();


	/* Checking to see if there was a proper value stored in the flash memory */
	if (low_count < 40 || low_count > 100) { // Is the low value saved okay?; if not, change it
		low_count = 50;
		eeprom_update_byte((void *) 100, low_count);
	}


	if (high_count < 40 || high_count > 100) { // Is the high value saved okay?; if not, change it
		high_count = 80;
		eeprom_update_byte((void *) 0, high_count);
	}

	timer0_init(); //timer 0 initialization

	timer1_init(); //timer 1 initialization

	timer2_init(); //timer 2 initialization

	_delay_ms(5);

    while (1) {              

		/* Displaying the Temp */

		// Celcius before conversion

		lcd_moveto(0,6);

		ds_temp(temp_data);

		int celsius_raw = (temp_data[1] << 8) + temp_data[0]; // Combines bytes

        // Convert the raw to the actual celsius
        celsius = celsius_raw * 9 / 8;

		// Conversion from celsius to farenheit

		int f_full = (celsius + 320) / 10; // Full values, ones and tens places
		int f_decimal = (celsius + 320) % 10; // Place to the right of the decimal


		current_temp = f_full; // Reference value for the LED and Buzzer


		/* Displaying the temp */
        char disp_temp[6];
		snprintf(disp_temp, 6, "%2d.%1d", f_full, f_decimal);
		lcd_stringout(disp_temp);

        ds_convert(); // Start next conversion



		/* Making the Servo Motor Turn */

		if (old_state == 0) { // When no button is pressed, it just tracks the temp on the motor

			OCR2A = 51 - (((4 * current_temp) / 10));

		}
		else if (old_state == 1) { // When Low button is pressed

			if (button_changed_recent == 1) { 
				

				OCR2A = 51 - (((4 * low_count) / 10)); // Takes the low count and moves the motor in that direction

				//Sets prescaler bits to 1024
				TCCR1B |= ((1 << CS12) | (1 << CS10));


			}
			else if (button_changed_recent == 0) {

				OCR2A = 51 - (((4 * current_temp) / 10));

			}
		}
		else if (old_state == 2) { // When the high button is pressed
			
			if (button_changed_recent == 1) {

				OCR2A = 51 - (((4 * high_count) / 10)); // Takes the high count and moves the motor in that direction

				//Sets prescaler bits to 1024
				TCCR1B |= ((1 << CS12) | (1 << CS10));

			}
			else if (button_changed_recent == 0) {

				OCR2A = 51 - (((4 * current_temp) / 10));

			}
			

		}
		
		
	


		/* Temp Button Pressed */

		// Changes the '=' to a '?' to indicate which value is being incremented

		if (button_change == 1) { // Was there a button press? 1 if yes.

			if (old_state == 0) { // Was in neither low or high

				if (new_state == 1) { // Now in low

					lcd_moveto(1,3);
					lcd_stringout("?");

				}

				else if (new_state == 2) { // Now in high

					lcd_moveto(1,12);
					lcd_stringout("?");

				}

			}
			else if (old_state == 1) { // Was in low 
				
				if (new_state == 2) { // Now in high

					lcd_moveto(1,12);
					lcd_stringout("?");

					lcd_moveto(1,3);
					lcd_stringout("=");

				}

			}
			else if (old_state == 2) { // Was in high

				if (new_state == 1) { // Now in low

					lcd_moveto(1,3);
					lcd_stringout("?");

					lcd_moveto(1,12);
					lcd_stringout("=");

				}

			}

			old_state = new_state; // The old state becomes the current (old) state to get ready for the next new_state

			// Resets the buttons so they can accurately read the next button press
			button_high = 0;
			button_low = 0;

			button_change = 0; // Sets flag back to 0

			button_changed_recent = 1; // Will Help with servo
			

		}

		
		/* Rotary Encoder updating */

		if (changed) {

			changed = 0;    // Reset changed flag

			if (old_state == 1) { // Low

				/* Outputs to the LCD */
				lcd_moveto(1,5);
				
				char disp_temp_rot_val[3];
				snprintf(disp_temp_rot_val, 3, "%2d", low_count);
				lcd_stringout(disp_temp_rot_val);

			}
			else if (old_state == 2) { // High

				/* Outputs to the LCD */
				lcd_moveto(1,14);
				
				char disp_temp_rot_val[3];
				snprintf(disp_temp_rot_val, 3, "%2d", high_count);
				lcd_stringout(disp_temp_rot_val);

			}

		
		}


		/* LED Change Colors */

		if (current_temp >= low_count && current_temp <= high_count) { // Temp in range, Green LED
			PORTC |= ((1 << 3) | (1 << 5));
			PORTC &= ~(1 << 4);
		}
		else if (current_temp < low_count && current_temp < high_count) { // Heater should turn on
			PORTC &= ~(1 << 3);
			PORTC |= ((1 << 4) | (1 << 5));
		}
		else if (current_temp > high_count && current_temp > low_count) { // Cooler should turn on
			PORTC &= ~(1 << 5);
			PORTC |= ((1 << 3) | (1 << 4));
		}
		

		/* Checking if the temp is 3 greater or less than to play */
		if (check_buzzer_played == 0) {

			if (current_temp >= high_count) {
				if (current_temp - high_count >= 3 || current_temp - high_count <= -3) {

					play_note(frequency[3]);
					
				}
			}
			else if (current_temp <= low_count) {
				if (current_temp - low_count >= 3 || current_temp - low_count <= -3) {

					play_note(frequency[3]);

				}
			}

		}
		else if (check_buzzer_played == 1) {

			if (current_temp < high_count && current_temp > low_count) { // Resets it for when the temp goes back in range
				check_buzzer_played = 0; // When it returns to range, reset so buzzer can be played when it goes out of range again
			}

		}

		_delay_ms(5);

    }
}



void play_note(uint16_t freq) { // Takes in a frequency

	uint32_t period;

    period = 1000000 / freq;    // Period of note in microseconds
	
	val = freq / 5;

	OCR0A = (16000000 / (2 * freq));

	TCCR0B = ((1 << CS02) | (1 << CS00));

}


void timer2_init(void) {

	// Timer 2 initialization
	TCCR2A |= (1 << WGM21);
	TCCR2A |= (1 << WGM20);
	TCCR2A |= (1 << WGM22);

	TCCR2A &= ~(1 << COM2A0);
	TCCR2A |= (1 << COM2A1);


	OCR2A = 35; // Maximum OCR value


	TCCR2B |= ((1 << CS22) | (1 << CS20) | (1 << CS21)); // Prescalar of 1024



}



/* To determine the value of the encoder as it is turned, ISR is for Group C */
ISR(PCINT1_vect) {


	rot_check = PINC; // Reads Pin C

	// Sets the values that are to be compared; either 1 or 0
	a = (rot_check & (1 << PC1)) >> 1;
	b = (rot_check & (1 << PC2)) >> 2;

	if (!b && !a)
	new_rot_state = 0;
	else if (!b && a)
	new_rot_state = 1;
	else if (b && a)
	new_rot_state = 2;
	else
	new_rot_state = 3;

	if (old_state == 1) { // Low

		if (old_rot_state == 0) {

			// Handle A and B inputs for state 0
			if (new_rot_state == 3) {
				low_count = low_count - 1;
			}
			else if (new_state == 1) {
				low_count = low_count + 1;
			}

		}
		else if (old_rot_state == 1) {

			// Handle A and B inputs for state 1
			if (new_rot_state == 0) {
				low_count = low_count - 1;
			}
			else if (new_rot_state == 2) {
				low_count = low_count + 1;
			}

		}
		else if (old_rot_state == 2) {

			// Handle A and B inputs for state 2
			if (new_rot_state == 1) {
				low_count = low_count - 1;
			}
			else if (new_rot_state == 3) {
				low_count = low_count + 1;
			}

		}
		else if (old_rot_state == 3) {   // old_state = 3

			// Handle A and B inputs for state 3
			if (new_rot_state == 2) {
				low_count = low_count - 1;
			}
			else if (new_rot_state == 0) {
				low_count = low_count + 1;
			}

		}

		eeprom_update_byte((void *) 100, low_count); // Updates the memory of the low count

	}
	else if (old_state == 2) { // High

		if (old_rot_state == 0) {

			// Handle A and B inputs for state 0
			if (new_rot_state == 3) {
				high_count = high_count - 1;
			}
			else if (new_state == 1) {
				high_count = high_count + 1;
			}

		}
		else if (old_rot_state == 1) {

			// Handle A and B inputs for state 1
			if (new_rot_state == 0) {
				high_count = high_count - 1;
			}
			else if (new_rot_state == 2) {
				high_count = high_count + 1;
			}

		}
		else if (old_rot_state == 2) {

			// Handle A and B inputs for state 2
			if (new_rot_state == 1) {
				high_count = high_count - 1;
			}
			else if (new_rot_state == 3) {
				high_count = high_count + 1;
			}

		}
		else if (old_rot_state == 3) {   // old_state = 3

			// Handle A and B inputs for state 3
			if (new_rot_state == 2) {
				high_count = high_count - 1;
			}
			else if (new_rot_state == 0) {
				high_count = high_count + 1;
			}

		}

		eeprom_update_byte((void *) 0, high_count); // Updates the memory of the high count

	}



	/* So that the high and low temps can't pass a certain value(s) */
	if (high_count > 100 || high_count < 40) {
		high_count = 100;
	}

	if (low_count < 40  || low_count > 100) {
		low_count = 40;
	}


	// Checking if changed, set flag to 1
	if (new_rot_state != old_rot_state) {
		old_rot_state = new_rot_state;
		changed = 1;
	}


}



// Pin Change Interrupt ISR for Group D
ISR(PCINT2_vect) {

	button_check = PIND;

	button_high = (button_check & (1 << PD3)) >> 3;
	button_low = (button_check & (1 << PD2)) >> 2;


	// High Button Pressed
	if (button_high && !button_low) {
		new_state = 2;
	}
	// Low Button Pressed
	else if (!button_high && button_low) {
		new_state = 1;
	}

	button_change = 1; // Flage that says a button was recenly pressed 




}


void timer0_init(void) { // Timer 0 function

	TCCR0A = (1 << WGM01); 

	TIMSK0 |= (1 << OCIE0A);

	PORTB &= ~(1 << 5);

}


ISR(TIMER0_COMPA_vect) {

	// Toggling bit 4
	if ((PORTB & (1 << PB5)) == 0) {
		PORTB |= (1 << PB5);    // Buzzer output high
	}
	else if ((PORTB & (1 << PB5)) != 0) {
		PORTB &= ~(1 << PB5);   // Buzzer output log
	}

	count2 = count2 + 1; // How many times the ISR has been run

	// Turns off the timer when val reaches the number of times it has to run
	if (count2 == val) {
		TCCR0B &= ~((1 << CS02) | (1 << CS00));
		count2 = 0;
	}

	check_buzzer_played = 1;

}


void timer1_init(void) { // Timer 1 init

	// Timer initialization
	TCCR1B |= ((1 << WGM12) | (1 << WGM13));
	TIMSK1 |= (1 << OCIE1A);

	// OCR1A value
	OCR1A = 62500;


}


ISR(TIMER1_COMPA_vect) { 

	// Turn off timer
	TCCR1B &= ~((1 << CS12) | (1 << CS10) | (1 << CS11));

	TCNT1 = 0; //Set clock to 0

	button_changed_recent = 0; // Says that the button was not pressed recently 

}