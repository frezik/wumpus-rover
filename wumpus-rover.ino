/*
Copyright (c) 2013, Timm Murray
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <Wire.h>
#include <Servo.h>

#define TWI_SLAVE_ADDR 0x10

#define SERVO_MIN_READING 0
#define SERVO_MAX_READING 1023
#define SERVO_MIN_DEGREES 0
#define SERVO_MAX_DEGREES 179
#define SERVO_PERCENT_THROTTLE 75
#define SERVO_STARTUP_WAIT_MS 10000
#define SERVO_MIN_PULSE_WIDTH 1000
#define SERVO_MAX_PULSE_WIDTH 1000
#define SERVO_MOTOR_PIN 2
#define SERVO_TURN_PIN 6

int led_green = 13;
int led_red   = 12;

Servo motor;
Servo turn;
unsigned long start_time;
int want_throttle = 10;
int want_turn     = 0;
int set_throttle  = 0;
int set_turn      = 0;
int set_led_green = LOW;
int set_led_red   = LOW;


void setup()
{
    Wire.begin( TWI_SLAVE_ADDR );
    Wire.onReceive( read_event );
    //Wire.onRequest( write_event );

    Serial.begin(9600);

    pinMode( led_green, OUTPUT );
    pinMode( led_red, OUTPUT );

    motor.attach( SERVO_MOTOR_PIN, SERVO_MIN_PULSE_WIDTH,
        SERVO_MAX_PULSE_WIDTH );
    turn.attach( SERVO_TURN_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH );
    start_time = millis();
}

void loop()
{
    unsigned long now_time = millis();
    unsigned long since_start = now_time - start_time;

    set_params_for_time( since_start );
    digitalWrite( led_green, set_led_green );
    digitalWrite( led_red, set_led_red );
    motor.write( set_throttle );
    turn.write( set_turn );
}

void read_event( int len )
{
    char preamble, payload_length, message_id;

    if( Wire.available() >= 4 ) {
        preamble       = (Wire.read() << 8) | Wire.read();
        payload_length = Wire.read();
        message_id     = Wire.read();

        Serial.print( "Got message ID: " );
        Serial.print( message_id );
    }
}

void write_event()
{
}

void set_params_for_time( long ms_since_start )
{
    if( SERVO_STARTUP_WAIT_MS > ms_since_start ) {
        set_throttle  = 0;
        set_turn      = 0;
        set_led_green = LOW;
        set_led_red   = HIGH;
    }
    else {
        set_throttle  = want_throttle;
        set_turn      = want_turn;
        set_led_green = HIGH;
        set_led_red   = HIGH;
    }
}
