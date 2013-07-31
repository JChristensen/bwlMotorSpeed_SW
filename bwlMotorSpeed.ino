//Motor speed control for Impression 5 BWL EV exhibit.
//Timer-interrupt-driven ADC sampling on one ADC channel.
//Reads potentiometer input on ADC channel 0 and
//uses PWM to drive a MOSFET on Arduino pin D3 (PD3, DIP pin 5).
//For Arduino Uno or equivalent @ 16MHz.
//Arduino IDE v1.0.5.
//Jack Christensen 13Jun2013

#include <Streaming.h>             //http://arduiniana.org/libraries/streaming/
#include <movingAvg.h>             //http://github.com/JChristensen/movingAvg

#define SAMPLE_SIZE 20
#define PAUSE_BETWEEN_SAMPLES 0    //milliseconds
#define BAUD_RATE 115200
#define PWM_OUT 3                  //pin for PWM motor drive

volatile boolean adcBusy;
volatile boolean timerFlag;
volatile int adcVal;

void setup(void)
{
    delay(1000);
    Serial.begin(BAUD_RATE);
    Serial << F("adcStart_us, adcLapse_us, adcValue") << endl;

    //set up the timer
    TCCR1B = 0;                //stop the timer
    TCCR1A = 0;
    TIFR1 = 0xFF;              //ensure all interrupt flags are cleared
    OCR1A = 6249;              //timer runs at 16MHz / 6250 / 256 (prescaler) = 10Hz (100ms between samples)
    OCR1B = 6249;
    cli();
    TCNT1 = 0;                 //clear the timer
    TIMSK1 = _BV(OCIE1B);      //enable timer interrupts
    sei();
    TCCR1B = _BV(WGM12) | _BV(CS12);    //start the timer, ctc mode, prescaler 256

    //set up the adc
    ADMUX = _BV(REFS0);                                //use AVcc as reference
    ADCSRA  = _BV(ADEN)  | _BV(ADATE) | _BV(ADIE);     //enable ADC, auto trigger, interrupt when conversion complete
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);    //ADC prescaler: divide by 128
    ADCSRB = _BV(ADTS2) | _BV(ADTS0);                  //trigger ADC on Timer/Counter1 Compare Match B
}

void loop(void)
{
    unsigned long tStart;            //timer interrupt time from micros()
    unsigned long tEnd;              //adc interrupt time from micros()
    int adc;
    movingAvg avgADC;
    
    while (!adcBusy);                //wait for next conversion to start
    tStart = micros();               //capture the time
    while (adcBusy);                 //wait for ADC conversion to complete
    tEnd = micros();                 //capture the time
    cli();
    adc = adcVal;                    //save the analog reading
    sei();
    analogWrite(PWM_OUT, avgADC.reading(adc / 4));
    
    //print the data
    //Serial << millis() << F(" ms, ") << tStart << F(", ") << tEnd - tStart << F(", ") << adc << endl;

    //Serial.flush();
    #if PAUSE_BETWEEN_SAMPLES > 0
    delay(PAUSE_BETWEEN_SAMPLES);
    #endif
}

ISR(ADC_vect)
{
    adcBusy = false;
    adcVal = ADC;
    //ADMUX = _BV(REFS0) | (++mux & 1);    //flip between mux0 and mux1
}

ISR(TIMER1_COMPB_vect)
{
    adcBusy = true;
}

