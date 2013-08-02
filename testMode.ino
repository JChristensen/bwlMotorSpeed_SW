//test mode timing constants
const unsigned long TEST_ON_DUR = 5000;              //ms on
const unsigned long TEST_OFF_DUR = 5000;             //ms off
const unsigned int TEST_REPEAT = 16;                 //number of on/off cycles
const unsigned long TEST_REST_DUR = 20000;           //ms to rest (off) between sets of on/off cycles
const unsigned long TEST_DURATION = 30;         //in minutes, e.g. 60 * 8 = 8 hours

enum TEST_STATES_t {TEST_INIT, TEST_ON, TEST_OFF, TEST_REST} TEST_STATE;

//returns false when TEST_DURATION has expired, else returns true.
//abort = true stops test prematurely.
boolean runTest(boolean abort)
{
    static unsigned long ms, msLast, testStart;
    static unsigned int nCycle;
    static unsigned int elapsed, prevElapsed;            //to track how long the test has run in minutes
    
    ms = millis();
    elapsed = (ms - testStart) / 60000UL;
    if (elapsed != prevElapsed) {
        prevElapsed = elapsed;
        Serial << F("Test mode: ") << elapsed << F(" of ") << TEST_DURATION << F(" minutes completed.") << endl;
    }
    
    if (abort) {
        analogWrite(PWM_OUT, 0);
        TEST_STATE = TEST_INIT;
        return false;
    }
    
    switch (TEST_STATE) {
        
        case TEST_INIT:
            nCycle = 0;
            testStart = ms;
            msLast = ms;
            prevElapsed = 65535;
            analogWrite(PWM_OUT, 255);
            TEST_STATE = TEST_ON;
            ++nCycle;
            break;
        
        case TEST_ON:
            if (ms - msLast >= TEST_ON_DUR) {
                msLast = ms;
                if (nCycle >= TEST_REPEAT) {
                    nCycle = 0;
                    analogWrite(PWM_OUT, 0);
                    TEST_STATE = TEST_REST;
                }
                else {
                    analogWrite(PWM_OUT, 0);
                    TEST_STATE = TEST_OFF;
                }
            }
            break;
        
        case TEST_OFF:
            if (ms - msLast >= TEST_OFF_DUR) {
                msLast = ms;
                analogWrite(PWM_OUT, 255);
                TEST_STATE = TEST_ON;
                ++nCycle;
            }
            break;
        
        case TEST_REST:
            if (ms - testStart >= TEST_DURATION * 60000UL) {
                analogWrite(PWM_OUT, 0);
                TEST_STATE = TEST_INIT;
                return false;
            }                
            else if (ms - msLast >= TEST_REST_DUR) {
                msLast = ms;
                analogWrite(PWM_OUT, 255);
                TEST_STATE = TEST_ON;
                ++nCycle;
            }
            break;
    }
    return true;
}
