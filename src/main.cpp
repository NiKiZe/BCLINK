
#include <Arduino.h>

#include <IRremote.hpp>

/* Pro Micro
https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/Pro_Micro_v13b.pdf
  o 2  PD1   PF4  21 o
  o 3  PD0   PF5  20 o
  o 4  PD4   PF6  19 o
  o 5  PC6   PF7  18 o
  o 6  PD7   PB1  15 o
  o 7  PE6   PB3  14 o
  o 8  PB4   PB2  16 o
  o 9  PB5   PB6  10 o
*/ 
uint8_t PB_MASK = B00001100;

void setPinModes(uint8_t mode) {
    pinMode(15, INPUT_PULLUP);
    pinMode(14, INPUT);
    pinMode(16, INPUT);
    pinMode(10, INPUT_PULLUP);
}

bool getPinState(uint8_t state, uint8_t pinMask) {
    return state & pinMask ? HIGH : LOW;
}

uint8_t olddata;
unsigned long nextprintmillis = 0;

uint8_t readState() {
    return (PINB & PB_MASK) >> 2;
}

#define uscalib -4
#define ushead 500+uscalib
#define usbit 640+uscalib

// Com C = 14 PORTB3
// Com D = 16 PORTB2
void sendBcLink(uint8_t portbpin, uint8_t* dataptr, uint8_t size) {
    uint8_t pinbit = _BV(portbpin);
    uint8_t i, mask, csum;
    csum = 0;
    // set csum in last byte
    for (i = 0; i < size-1; i++) {
        csum += *(dataptr + i);
    }
    *(dataptr + size - 1) = ~csum & 0xff;

    PORTB &= ~pinbit; // low
    DDRB |= pinbit; // set output
    delayMicroseconds(ushead);

    for (i = 0; i < size; i++) {
        mask = 0x80;
        while (mask) {
            if (*(dataptr + i) & mask)
                PORTB |= pinbit; // high
            else
                PORTB &= ~pinbit; // low
            delayMicroseconds(usbit); // adjust to get ~640us per pulse
            mask = mask >> 1;
        }
    }

    // TODO csum
    PORTB ^= pinbit; // invert last sent bit
    delayMicroseconds(usbit);
    PORTB &= ~pinbit; // low - trailing low
    delayMicroseconds(ushead);
    DDRB &= ~pinbit; // set input
}

void setup() {
    Serial.begin(0);
    while (!Serial)
    delay(500);
    
    uint8_t tone[] = {0x84, 0x00, 0x11, 0x00};
    sendBcLink(PORTB3, tone, sizeof(tone));
    delay(100);

    uint8_t tone2[] = {0x80, 0x8E, 0x00}; // 8E B3 B3
    sendBcLink(PORTB3, tone2, sizeof(tone2));
    delay(1000);
    tone2[1] = 0xB3;
    sendBcLink(PORTB3, tone2, sizeof(tone2));
    delay(1000);
    tone2[1] = 0x10;
    sendBcLink(PORTB3, tone2, sizeof(tone2));
    delay(100);

    uint8_t led[] = {0x81, 0x83, 0x00};
    // 20, 40, 80
    sendBcLink(PORTB3, led, sizeof(led));
    delay(1000);

    /*for (uint8_t i = 0; i < 0xff; i++) {
        // Orange, Red, Green?
        // on/off/flash speed
        led[1] = i;
        Serial.println(i, HEX);
        sendBcLink(PORTB3, led, sizeof(led));
        delay(1000);
    }*/

    // Send B button from Exit
    uint8_t but[] = {0x89, 0x0B, 0x00};
    sendBcLink(PORTB2, but, sizeof(but));
    delay(100);

    // Send Tag
    uint8_t tag[] = {0x8A, 0x97, 0x97, 0x97, 0x97, 0x00};
    sendBcLink(PORTB2, tag, sizeof(tag));
    delay(100);

    but[1] = 0xC;
    sendBcLink(PORTB2, but, sizeof(but));
    delay(100);
    but[1] = 0xf;
    sendBcLink(PORTB2, but, sizeof(but));
    delay(100);

    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
    IrReceiver.begin(16);

    // infos for receive
    Serial.print(RECORD_GAP_MICROS);
    Serial.println(F(" us is the (minimum) gap, after which the start of a new IR packet is assumed"));
    Serial.print(MARK_EXCESS_MICROS);
    Serial.println();

    setPinModes(INPUT);
}

void printMillis(unsigned long cmillis)
{
    unsigned long seconds = cmillis / 1000;
    cmillis %= 1000;
    unsigned long minutes = seconds / 60;
    seconds %= 60;
    unsigned long hours = minutes / 60;
    minutes %= 60;
    unsigned long days = hours / 24;
    hours %= 24;
    if (days != 0) {
        Serial.print(days);
        Serial.print("d ");
    }
    Serial.print(hours);
    Serial.print(':');
    if (minutes < 10)
        Serial.print('0');
    Serial.print(minutes);
    Serial.print(':');
    if (seconds < 10)
        Serial.print('0');
    Serial.print(seconds);
    Serial.print('.');
    if (cmillis < 100)
        Serial.print('0');
    if (cmillis < 10)
        Serial.print('0');
    Serial.print(cmillis);
}

void printData(uint8_t data, uint8_t olddata) {
        bool RC = getPinState(data, 1 << 1);
        bool RD = getPinState(data, 1 << 0);
        Serial.print('\t');
        Serial.print(RC ? 'C' : ' ');
        Serial.print(RD ? 'D' : ' ');
        Serial.print('\t');
        Serial.print(data ^ olddata, 2);
}

unsigned long lastus = 0;
void loop() {
    if (IrReceiver.decode()) {  // Grab an IR code
        // At 115200 baud, printing takes 200 ms for NEC protocol and 70 ms for NEC repeat
        Serial.println(); // blank line between entries
        // Check if the buffer overflowed
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
            IrReceiver.printIRResultShort(&Serial);
            Serial.println(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of "));
            // see also https://github.com/Arduino-IRremote/Arduino-IRremote#compile-options--macros-for-this-library
        } else {
            /*
            Serial.print(F("Result as internal 8bit ticks (50 us) array - compensated with MARK_EXCESS_MICROS="));
            Serial.println(MARK_EXCESS_MICROS);
            */
            IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, false);
            /*
            Serial.print(F("microseconds array - "));
            IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, true);
            IrReceiver.printIRResultMinimal(&Serial);
            Serial.print("  ");
            */
            Serial.println(IrReceiver.decodedIRData.decodedRawData, 2);

        }
        IrReceiver.resume();                            // Prepare for the next value
    }


    return;
    unsigned long us;
    int lasti = 0;
    for (int i = 0; i <= 100000; i++) {
        uint8_t data = readState();
        if (data == olddata) {
            if (i - lasti > 4000)
                break;
            continue;
        }
        us = micros();
        int usdiff = us - lastus;
        Serial.print(usdiff);
        Serial.print("us\t");
        Serial.print(i - lasti);
        lastus = us;
        lasti = i;
        printData(data, olddata);
        Serial.print(' ');
        Serial.print(usdiff / 600);
        Serial.println();
        olddata = data;
    }
}
