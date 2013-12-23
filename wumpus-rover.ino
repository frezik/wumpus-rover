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

#define TWI_SLAVE_ADDR 0x09
#define THROTTLE_REGISTER 0x01
#define TURN_REGISTER     0x02

#define SERVO_PERCENT_THROTTLE 75
#define SERVO_STARTUP_WAIT_MS 2000
#define SERVO_MIN_PULSE_WIDTH 1000
#define SERVO_MAX_PULSE_WIDTH 1000
#define ZERO_THROTTLE 1500
#define ZERO_TURN 79
#define SERVO_MOTOR_PIN 2
#define SERVO_TURN_PIN 6

#define CHANNEL_THROTTLE 0
#define CHANNEL_TURN 1

#define SLEEP_MS 15

#define DEBUG 0

int led_green = 13;
int led_red   = 12;

Servo motor;
Servo turn;
unsigned long start_time;
int want_throttle = 1700;
int want_turn     = 0;
int set_throttle  = 0;
int set_turn      = 0;
int set_led_green = LOW;
int set_led_red   = LOW;


void setup()
{
    Serial.begin(9600);
    if( DEBUG ) Serial.println( "[SETUP] Started init" );

    Wire.begin( TWI_SLAVE_ADDR );
    Wire.onReceive( read_i2c );
    Wire.onRequest( write_i2c );

    pinMode( led_green, OUTPUT );
    pinMode( led_red, OUTPUT );

    motor.attach( SERVO_MOTOR_PIN );
    turn.attach( SERVO_TURN_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH );

    if( DEBUG ) Serial.println( "[SETUP] Finished init" );
    start_time = millis();
}

void loop()
{
    unsigned long now_time = millis();
    unsigned long since_start = now_time - start_time;

    //Serial.println( "[LOOP] Starting loop . . . " );

    set_params_for_time( since_start );
    digitalWrite( led_green, set_led_green );
    digitalWrite( led_red, set_led_red );

    if( DEBUG ) {
        Serial.print( "[LOOP] Setting Throttle " );
        Serial.print( set_throttle );
        Serial.print( ", Turn: " );
        Serial.println( set_turn );
    }

    motor.writeMicroseconds( set_throttle );
    turn.write( set_turn );

    //Serial.println( "[LOOP] Done with loop " );
    delay( SLEEP_MS );
}

void read_i2c( int len )
{
    if( len >= 3 ) {
        int register_index = Wire.read();

        switch( register_index ) {
            case THROTTLE_REGISTER:
                want_throttle = (Wire.read() << 8) | Wire.read();
                break;
            case TURN_REGISTER:
                want_turn = (Wire.read() << 8) | Wire.read();
                break;
        }
    }
}

void write_i2c()
{
    Wire.write( 1 );
}

void set_params_for_time( long ms_since_start )
{
    if( SERVO_STARTUP_WAIT_MS > ms_since_start ) {
        if( DEBUG ) Serial.println( "[LOOP] Servo setup loop . . . " );

        set_throttle  = ZERO_THROTTLE;
        set_turn      = ZERO_TURN;
        set_led_green = LOW;
        set_led_red   = HIGH;
    }
    // TODO Don't set throttle or turn if the last packet came more than 
    // a certain time ago (1 second?)
    else {
        set_throttle  = want_throttle;
        set_turn      = want_turn;
        set_led_green = HIGH;
        set_led_red   = HIGH;
    }
}

int read_16bits_from_wire()
{
    int read = (Wire.read() << 8) | Wire.read();
    return read;
}
