/*
 * Laboratorio 5 
 *
 * 
 * Author : Nesthor Guillermo
 */ 
// Frecuencia del reloj
#define F_CPU 16000000

// Librerias no internas
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Libreria_PWManual/PWM1.h"

void setup(void);

int main(void)
{
	setup();
	while (1)
	{
		// Todo se maneja en las interrupciones
		_delay_ms(1);
	}
}

void setup(void)
{
	cli(); // Deshabilitar interrupciones globales

	// Configurar ADC6 (PC6), ADC7 (PC7) y ADC5 (PC5) como entradas
	DDRC &= ~((1 << PC6) | (1 << 7) | (1 << PC5));  // PC5, PC6 y PC7 como entradas
	PORTC &= ~((1 << PC6) | (1 << 7) | (1 << PC5)); // Sin pull-up

	// Configurar PD4 como salida para el LED de PWM manual
	DDRD |= (1 << PD4);
	PORTD &= ~(1 << PD4);

	initTimer1Servo(); // Configurar Timer1 para servos
	initTimer2PWM();   // Configurar Timer2 para PWM manual
	initADC();
	
	sei(); // Habilitar interrupciones globales
}