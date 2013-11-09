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

#define SLAVE_ADDR 0x10

int led_green = 13;
int led_red   = 12;


void setup()
{
    Wire.begin( SLAVE_ADDR );
    Wire.onReceive( read_event );
    //Wire.onRequest( write_event );

    Serial.begin(9600);

    pinMode( led_green, OUTPUT );
    pinMode( led_red, OUTPUT );
}

void loop()
{
    digitalWrite( led_green, HIGH );
    digitalWrite( led_red,   LOW  );
    delay( 1000 );
    digitalWrite( led_green, LOW  );
    digitalWrite( led_red,   HIGH );
    delay( 1000 );
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
