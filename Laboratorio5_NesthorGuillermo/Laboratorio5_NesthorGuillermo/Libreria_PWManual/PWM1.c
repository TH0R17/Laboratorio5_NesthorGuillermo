/*
 * PWM1.c
 *
 * 
 *  Author: Nesthor
 */ 
#include "PWM1.h"
#include <avr/interrupt.h>

// Variables globales
uint16_t adc6_value = 0;  // Valor de ADC6 (PC6)
uint16_t adc7_value = 0;  // Valor de ADC7 (ADC7)
uint8_t current_channel = 6; // Canal ADC actual (6 o 7)

// Variables para el PWM manual
volatile uint8_t contador_pwm = 0;       // Contador para PWM manual
volatile uint8_t pwm_ciclo = 128;        // Valor de umbral inicial (50%)
volatile uint8_t Estado_pwm = 1;         // Estado inicial del LED (encendido)

void initTimer1Servo(void)
{
	// Configurar PB1 (OC1A) y PB2 (OC1B) como salidas para servos
	DDRB |= (1 << PB1) | (1 << PB2); // PB1 = OC1A, PB2 = OC1B
	
	// Modo PWM, Phase Correct, 10-bit
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << CS11); // Prescaler 8
	
	// Frecuencia PWM ~50Hz (Periodo 20ms)
	ICR1 = 39999; // (16MHz / (8 * 50Hz)) - 1 = 39999
	
	// Valores iniciales (posición central)
	OCR1A = SERVO_CENTER * 2; // Convertir microsegundos a ticks
	OCR1B = SERVO_CENTER * 2;
}

void initTimer2PWM(void)
{
	// Configurar Timer2 para interrupción cada 32us (aproximadamente 31.25kHz de frecuencia base)
	TCCR2A = 0; // Modo normal
	TCCR2B = (1 << CS21); // Prescaler 8 (16MHz/8 = 2MHz, tick cada 0.5us)
	TIMSK2 = (1 << TOIE2); // Habilitar interrupción por overflow
	TCNT2 = 0; // Iniciar contador en 0
	
	// El overflow ocurrirá cada 256 ticks (128us) pero lo ajustaremos para 64 ticks (32us)
	OCR2A = 64; // Valor de comparación
	TCCR2A |= (1 << WGM21); // Modo CTC
	TIMSK2 |= (1 << OCIE2A); // Habilitar interrupción por comparación
}

void initADC(void)
{
	// Referencia AVcc, ajuste a la izquierda, comenzar con ADC6
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX2) | (1 << MUX1);

	// Habilitar ADC, interrupción del ADC, prescaler de 128
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) |
	(1 << ADEN) | (1 << ADIE);

	// Iniciar primera conversión
	ADCSRA |= (1 << ADSC);
}

// Interrupción del ADC
ISR(ADC_vect)
{
	if(current_channel == 6)
	{
		adc6_value = ADCH;  // Leer valor de ADC6 (0-255)
		
		// Mapear valor ADC a rango servo (1000-2000)
		uint16_t servoPos = SERVO_MIN + ((uint32_t)adc6_value * (SERVO_MAX - SERVO_MIN)) / 255;
		OCR1A = servoPos; // Convertir microsegundos a ticks (0.5us por tick)
		
		// Cambiar a ADC7 para la próxima lectura
		ADMUX = (ADMUX & 0xF0) | 0x07;
		current_channel = 7;
	}
	else if(current_channel == 7)
	{
		adc7_value = ADCH;  // Leer valor de ADC7 (0-255)
		
		// Mapear valor ADC a rango servo (1000-2000)
		uint16_t servoPos = SERVO_MIN + ((uint32_t)adc7_value * (SERVO_MAX - SERVO_MIN)) / 255;
		OCR1B = servoPos; // Convertir microsegundos a ticks
		
		// Cambiar a ADC5 para leer el tercer potenciómetro
		ADMUX = (ADMUX & 0xF0) | 0x05;
		current_channel = 5;
	}
	else
	{
		// Leer valor del tercer potenciómetro (ADC5)
		pwm_ciclo = ADCH; // Usar el valor directamente como umbral PWM
		
		// Volver a ADC6 para la próxima lectura
		ADMUX = (ADMUX & 0xF0) | 0x06;
		current_channel = 6;
	}
	
	// Iniciar nueva conversión
	ADCSRA |= (1 << ADSC);
}

// Interrupción del Timer2 para PWM manual
ISR(TIMER2_COMPA_vect)
{
	contador_pwm++;
	
	if(contador_pwm == 0)
	{
		// Cuando el contador vuelve a 0, encender el LED
		PORTD |= (1 << PD4);
		Estado_pwm = 1;
	}
	else if(contador_pwm >= pwm_ciclo && Estado_pwm)
	{
		// Cuando alcanza el umbral, apagar el LED
		PORTD &= ~(1 << PD4);
		Estado_pwm = 0;
	}
}