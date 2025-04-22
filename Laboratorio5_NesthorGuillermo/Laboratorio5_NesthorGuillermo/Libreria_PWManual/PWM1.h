/*
 * PWM1.h
 *
 * 
 *  Author: Nesthor
 */ 

#ifndef PWM1_H_
#define PWM1_H_

#include <avr/io.h>
#include <stdint.h>

// Definiciones públicas
#define SERVO_MIN 800
#define SERVO_MAX 3200
#define SERVO_CENTER 1600
#define ADC_MIN 0
#define ADC_MAX 255

// Variables públicas
extern uint16_t adc6_value;
extern uint16_t adc7_value;
extern volatile uint8_t pwm_ciclo;

// Prototipos de funciones
void initTimer1Servo(void);
void initTimer2PWM(void);
void initADC(void);

#endif