//Motor speed control for Impression 5 BWL EV exhibit.
//Timer-driven ADC sampling on one ADC channel.
//Reads potentiometer input on ADC channel 0 and
//uses PWM to drive a MOSFET on Arduino pin D3 (PD3, DIP pin 5).
//For Arduino Uno or equivalent @ 16MHz.
//Arduino IDE v1.0.5.
//Jack Christensen 13Jun2013
//v1.1 31Jul2013 -- Added automatic pedal calibration, quantization of PWM output.

#include <Streaming.h>                  //http://arduiniana.org/libraries/streaming/
#include <movingAvg.h>                  //http://github.com/JChristensen/movingAvg
#include <avr/eeprom.h>

#define SAMPLE_SIZE 20
#define PAUSE_BETWEEN_SAMPLES 0         //milliseconds
#define BAUD_RATE 115200
#define PWM_OUT 3                       //pin for PWM motor drive
#define CALIBRATION_INTERVAL 10000      //milliseconds at startup for pedal calibration
#define CALIB_DIFF_MIN 100              //minimum difference in ADC readings to recalibrate pedals
#define CALIB_SAVE_MIN 5                //minimum difference to resave a limit in EEPROM

volatile boolean adcBusy;
volatile int accelVal;
uint16_t accel, accelAvg, accelMin, accelMax, accelNewMin=9999, accelNewMax;
uint16_t EEMEM EEP_accelMin, EEP_accelMax;

void setup(void)
{
    delay(1000);
    Serial.begin(BAUD_RATE);

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
    
    //pedal calibration
    Serial << F("***PEDAL CALIBRATION") << endl;
    while (millis() <= CALIBRATION_INTERVAL) {
        while (!adcBusy);               //wait for next conversion to start
        while (adcBusy);                //wait for ADC conversion to complete
        cli();
        accel = accelVal;               //save the analog reading
        sei();
        if (accel < accelNewMin) accelNewMin = accel;
        if (accel > accelNewMax) accelNewMax = accel;
    }
    
    //get the saved values from eeprom
    accelMin = eeprom_read_word(&EEP_accelMin);
    accelMax = eeprom_read_word(&EEP_accelMax);
    Serial << F("Saved accel limits: ") << accelMin << ' ' << accelMax << endl;
    Serial << F("New accel calibration values: ") << accelNewMin << ' ' << accelNewMax << endl;
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
}

void loop(void)
{
    unsigned long tStart;               //timer interrupt time from micros()
    unsigned long tEnd;                 //adc interrupt time from micros()
    movingAvg avgAccel;
    
    while (!adcBusy);                   //wait for next conversion to start
//    tStart = micros();                  //capture the time
    while (adcBusy);                    //wait for ADC conversion to complete
//    tEnd = micros();                    //capture the time
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

    #if PAUSE_BETWEEN_SAMPLES > 0
    delay(PAUSE_BETWEEN_SAMPLES);
    #endif
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
