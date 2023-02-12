/* Copyright (C) 2020 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */


#include <Arduino.h>

#define STATUS0 11
#define STATUS1 13

#define A_HIGH_SENSE 2
#define A_HIGH 5
#define A_LOW_SENSE 8
#define A_LOW A3

#define B_HIGH_SENSE 3
#define B_HIGH 7
#define B_LOW_SENSE 9
#define B_LOW A4

#define C_HIGH_SENSE 4
#define C_HIGH 10
#define C_LOW_SENSE A2
#define C_LOW A5

#define VOLTAGE_SENSE A6
#define CURRENT_SENSE A7

#define A_VOLTAGE_SENSE A0
#define A_CURRENT_SENSE A1

volatile uint16_t fcount;
uint16_t duty, duty_dir = 1;
const uint16_t max_current = 500, brake_current = 300, max_voltage = 1400; // hundredths of volts/amps

inline void on() {
    // all high of
    PORTB &= ~_BV(PB2);
    PORTD &= ~(_BV(PD5) | _BV(PD7));

    // output on high
    DDRB |= _BV(PB2);
    DDRD |= _BV(PD5) | _BV(PD7);
    
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");

    // output on low and brake
    PORTC |= _BV(PC3) | _BV(PC4) | _BV(PC5);    
    DDRC |= _BV(PC3) | _BV(PC4) | _BV(PC5);    
}

inline void off() {
    // do not drive low mosfets
    DDRC &= ~(_BV(PC4) | _BV(PC5) | _BV(PC6));
    PORTC &= ~(_BV(PC4) | _BV(PC5) | _BV(PC6));

    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");

    // do not drive high mosfets
    DDRB &= ~_BV(PB2);
    DDRD &= ~(_BV(PD5) | _BV(PD7));
}

void frequency_counter()
{
    fcount++;
}

void setup()
{
    PCICR = 0;
    MCUSR = 0;

    DDRB|=_BV(PB3) | _BV(PB5);
    PORTB|=_BV(PB3) | _BV(PB5);

    pinMode(STATUS0, OUTPUT);
    digitalWrite(STATUS0, HIGH);

    pinMode(STATUS1, OUTPUT);
    digitalWrite(STATUS1, LOW);

    attachInterrupt(0, frequency_counter, RISING);

    ADSR = _BV(ACBG) | _BV(ACIE);

    TIMSK1 = 0;
    TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
    TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS10); //PRESCALER=0 MODE 14(FAST PWM)
    OCR1A=1000; // duty
    ICR1=8000; // 1khz

    //DIDR1 |= _BV(AIN1D);

    Serial.begin(38400);
    Serial.println(F("Startup"));
}

ISR(ANA_COMP_vect)
{
    TIMSK1 = 0;  // stop modulation!
    on();
}

//ISR(TIMER1_OVF_vect) __attribute__((naked));
ISR(TIMER1_OVF_vect)
{
    on();
}

//ISR(TIMER1_COMPA_vect) __attribute__((naked));
ISR(TIMER1_COMPA_vect)
{
    off();
}

void emergency_brake()
{
    TIMSK1 = 0;  // stop modulation!

    digitalWrite(STATUS1, 1);
    on();

    Serial.println(F("EMERGENCY BRAKE"));
    int count = 0;
    for(;;) {
        uint16_t voltage = analogRead(VOLTAGE_SENSE)*3300/1023;
        uint16_t current = analogRead(CURRENT_SENSE)*2500/1023;
        if(voltage < max_voltage && current < 100)
            break;
        delay(50);
        count++;
    }
    Serial.println(F("RESET"));

    
    delay(5000);
    Serial.print(F("RESUME: "));
    Serial.println(count);

    duty = 0;
    
    off();
    digitalWrite(STATUS1, 0);
}

void loop()
{
    // in hundreths of volts/amps
    uint16_t voltage = analogRead(VOLTAGE_SENSE)*3300/1023;
    uint16_t current = analogRead(CURRENT_SENSE)*2500/1023;

    uint16_t a_voltage = analogRead(A_VOLTAGE_SENSE)*3300/1023;
    
    //   x = analogRead(A_CURRENT_SENSE);
//    float a_current = x*25/1023.0;

    int ac_max = 0; //ACSR & _BV(ACO);
    if(current > max_current || voltage > max_voltage || ac_max) {
        emergency_brake();
        return;
    }
    
    cli();
    uint16_t count = fcount;
    fcount = 0;
    sei();
    static uint16_t last_freq_t;
    uint16_t t = millis();
    uint16_t dt = t-last_freq_t;
    last_freq_t = t;
    uint16_t frequency = count * 1000 / dt;
    
    Serial.print(F("V "));
    Serial.print(voltage);
    
    Serial.print(F(" A "));
    Serial.print(current);

    //Serial.print(F(" AV "));
    //Serial.print(a_voltage);

    Serial.print(F(" F "));
    Serial.print(frequency);

    Serial.print(F(" D "));
    Serial.println(duty);
     
    static uint16_t last_t;
    static uint8_t status;
    dt = t - last_t;
    if(dt > 100) { // 100 ms
        uint8_t status1 = LOW;
        last_t = t;

        static uint16_t last_current;
        int16_t current_d = (current - last_current) * 1000 / dt;            
        last_current = current;

        static uint16_t last_voltage;
        int16_t voltage_d = (voltage - last_voltage) * 1000.0f / dt;            
        last_voltage = voltage;

        int16_t dc = current - brake_current;
        int16_t pdc = (dc>>2) + current_d; // will hit brake current in 4 seconds

        int16_t dv = voltage - max_voltage;
        int16_t pdv = (dv>>2) + voltage_d; // will hit max voltage in 4 seconds
        if(pdc > 0) {
            // need to brake from too much current?
            duty += pdc;
            status1 = !status1;
        } else if(pdv > 0) {
            // if approaching max voltage, brake
            duty += pdv;
            status1 = !status1;
        } else {
            // MPPT strategy
            if(current_d < 0)
                duty_dir = -duty_dir;

            duty += duty_dir;

            // assume turbine, apply current feedback
            uint16_t curadj = sqrt(1+current_d/current);
            duty *= curadj;
 
            status1 = 0;

            if(current < 10) { // current < 100mA
                if(duty > 10) // limit duty at low currents
                    duty = 10;
            }
            if(duty < 0)
                duty = 0;
            else if(duty > 60)
                emergency_brake();
        }

        // not currently modulating
        if(duty < 5) {
            TIMSK1 = 0;
            off();
        } else {
            TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);
            OCR1A = duty * 80;
        }
        digitalWrite(STATUS1, status1);
    } else
        delay(10);
}
