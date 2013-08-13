//Motor speed control for Impression 5 BWL EV exhibit.
//Timer-driven ADC sampling on one ADC channel.
//Reads potentiometer input on ADC channel 0 and
//uses PWM to drive a MOSFET on Arduino pin D3 (PD3, DIP pin 5).
//For Arduino Uno or equivalent @ 16MHz.
//Arduino IDE v1.0.5.
//Jack Christensen 13Jun2013
//v1.1 31Jul2013 -- Added automatic pedal calibration, quantization of PWM output.
//v1.2 01Aug2013 -- Change to state machine architecture. Add button to select modes.

#include <avr/eeprom.h>
#include <Button.h>                     //http://github.com/JChristensen/Button
#include <movingAvg.h>                  //http://github.com/JChristensen/movingAvg
#include <Streaming.h>                  //http://arduiniana.org/libraries/streaming/

//pin definitions
const int PWM_OUT = 3;                  //PWM for MOSFET motor drive
const int RED_LED = 8;                  //PB0
const int GRN_LED = 9;                  //PB1
const int MODE_BUTTON = 10;

//fast port manipulation macros for LEDs
//see atmel datasheet for register (port) names: http://goo.gl/coZTO4
//see avr-libc user's manual for _BV() macro:
//http://www.nongnu.org/avr-libc/
//http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_use_bv
#define RED_LED_ON PORTB |= _BV(PORTB0)
#define RED_LED_OFF PORTB &= ~_BV(PORTB0)
#define RED_LED_TOGGLE PINB |= _BV(PORTB0)
#define GRN_LED_ON PORTB |= _BV(PORTB1)
#define GRN_LED_OFF PORTB &= ~_BV(PORTB1)
#define GRN_LED_TOGGLE PINB |= _BV(PORTB1)

//other constants
const long BAUD_RATE = 115200;
const int CALIB_DIFF_MIN = 100;                      //minimum difference in ADC readings needed to recalibrate pedals
const int CALIB_SAVE_MIN = 3;                        //minimum difference in a single calibration values to resave in EEPROM
const boolean BTN_PULLUP = true;                     //turn on pullup resistor for buttons
const boolean BTN_INVERT = true;                     //inverted logic for buttons with pullups
const unsigned long BTN_DEBOUNCE = 25;               //milliseconds
const unsigned long SLOW_BLINK = 1000;               //milliseconds
const unsigned long FAST_BLINK = 100;                //milliseconds

//globals
volatile boolean adcBusy;
volatile int accelVal;
uint16_t EEMEM EEP_accelMin, EEP_accelMax;
    Button btnMode(MODE_BUTTON, BTN_PULLUP, BTN_INVERT, BTN_DEBOUNCE);    //mode button

void setup(void)
{
    delay(1000);
    Serial.begin(BAUD_RATE);
    
    //pin initialization
    pinMode(RED_LED, OUTPUT);
    pinMode(GRN_LED, OUTPUT);

    //set up the timer
    TCCR1B = 0;                         //stop the timer
    TCCR1A = 0;
    TIFR1 = 0xFF;                       //ensure all interrupt flags are cleared
    OCR1A = 6249;                       //timer runs at 16MHz / 6250 / 256 (prescaler) = 10Hz (100ms between samples)
    OCR1B = 6249;
    cli();
    TCNT1 = 0;                          //clear the timer
    TIMSK1 = _BV(OCIE1B);               //enable timer interrupts
    sei();
    TCCR1B = _BV(WGM12) | _BV(CS12);    //start the timer, ctc mode, prescaler 256

    //set up the adc
    ADMUX = _BV(REFS0);                                //use AVcc as reference
    ADCSRA  = _BV(ADEN)  | _BV(ADATE) | _BV(ADIE);     //enable ADC, auto trigger, interrupt when conversion complete
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);    //ADC prescaler: divide by 128
    ADCSRB = _BV(ADTS2) | _BV(ADTS0);                  //trigger ADC on Timer/Counter1 Compare Match B
}

enum STATES_t {INIT, CALIBRATE, TEST, RUN, STOP} STATE;

void loop(void)
{
    static uint16_t accel, accelAvg, accelMin, accelMax, accelNewMin=9999, accelNewMax;
    static unsigned long ms, msLast;
    movingAvg avgAccel;
    
    ms = millis();
    btnMode.read();
    
    switch (STATE) {
        
        case INIT:
            //get the saved pedal calibration values from eeprom
            accelMin = eeprom_read_word(&EEP_accelMin);
            accelMax = eeprom_read_word(&EEP_accelMax);
        
            if (btnMode.isPressed()) {
                STATE = CALIBRATE;
                RED_LED_ON;
                GRN_LED_OFF;
                while (btnMode.isPressed()) btnMode.read();    //wait for user to release the button
                Serial << F("***PEDAL CALIBRATION") << endl;
                Serial << F("Existing accel limits: ") << accelMin << ' ' << accelMax << endl;
            }
            else {
                STATE = RUN;
                RED_LED_OFF;
                GRN_LED_ON;
            }
            break;
        
        case CALIBRATE:                     //pedal calibration
            if (btnMode.wasReleased()) {
                Serial << F("New accel limits from calibration: ") << accelNewMin << ' ' << accelNewMax << endl;
                if (accelNewMax - accelNewMin >= CALIB_DIFF_MIN) {
                    if (abs(accelNewMin - accelMin) >= CALIB_SAVE_MIN) {
                        accelMin = accelNewMin;
                        eeprom_write_word(&EEP_accelMin, accelMin);
                        Serial << F("accelMin updated") << endl;
                    }
                    if (abs(accelNewMax - accelMax) >= CALIB_SAVE_MIN) {
                        accelMax = accelNewMax;
                        eeprom_write_word(&EEP_accelMax, accelMax);
                        Serial << F("accelMax updated") << endl;
                    }
                }
                else {
                    Serial << F("Invalid accel calibration input, no change.") << endl;
                }
                Serial << F("***END CALIBRATION") << endl;
                RED_LED_OFF;
                GRN_LED_ON;
                STATE = RUN;
            }
            else {
                while (!adcBusy);               //wait for next conversion to start
                while (adcBusy);                //wait for ADC conversion to complete
                cli();
                accel = accelVal;               //save the analog reading
                sei();
                if (accel < accelNewMin) accelNewMin = accel;
                if (accel > accelNewMax) accelNewMax = accel;
                RED_LED_TOGGLE;
            }
            break;
            
        case TEST:
            if (!runTest(false)) {
                STATE = STOP;
                GRN_LED_ON;
                RED_LED_OFF;
            }
            if (ms - msLast >= FAST_BLINK) {
                msLast = ms;
                GRN_LED_TOGGLE;
                RED_LED_TOGGLE;
            }
            if (btnMode.wasReleased()) {
                runTest(true);            //abort test mode
                GRN_LED_ON;
                RED_LED_OFF;
                STATE = RUN;
            }
            break;
            
        case RUN:
            while (!adcBusy);                   //wait for next conversion to start
            while (adcBusy);                    //wait for ADC conversion to complete
            cli();
            accel = accelVal;                   //save the analog reading
            sei();
            accelAvg = avgAccel.reading(accel);
            //quantize the accelAvg value to 17 steps: 0, 15, 31, 47, 63, 79, ... 223, 239, 255.
            accelAvg = constrain(map(accelAvg, accelMin, accelMax, 0, 17), 0, 16);
            if (accelAvg > 0) accelAvg = accelAvg * 16 - 1;
            analogWrite(PWM_OUT, accelAvg);
        //    Serial << accel << ' ' << accelAvg << endl;
        //    Serial.flush();
            GRN_LED_TOGGLE;
            if (btnMode.wasReleased()) {
                GRN_LED_ON;
                RED_LED_ON;
                msLast = ms;
                STATE = TEST;
            }
            break;
        
        case STOP:    //Abandon all hope, ye who enter here (no exit from STOP state but a reset)
            if (ms - msLast >= SLOW_BLINK) {
                msLast = ms;
                GRN_LED_TOGGLE;
                RED_LED_TOGGLE;
            }
            break;
    }
}

ISR(ADC_vect)
{
    adcBusy = false;
    accelVal = ADC;
    //ADMUX = _BV(REFS0) | (++mux & 1);     //flip between mux0 and mux1
}

ISR(TIMER1_COMPB_vect)
{
    adcBusy = true;
}
