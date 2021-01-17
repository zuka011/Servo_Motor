#include "Arduino.h"
#include "Servo_Motor.h"

static const uint8_t MAX_SERVOS = 4;
static ServoMotor *callback_servos[MAX_SERVOS];

static uint8_t last_servo = 0;

ServoMotor::ServoMotor() {
    
    controlPin = -1;
    pulseTimer = 0;
    sweepMode = false;
}

ServoMotor::~ServoMotor() {
    disableCallback();
}

bool ServoMotor::attach(int controlPin, int lowConstraint = 0, int highConstraint = 180) {

    if(highConstraint < lowConstraint) {
        uint8_t temp = highConstraint;
        highConstraint = lowConstraint;
        lowConstraint = temp;
    }

    if(controlPin < 0 || lowConstraint < 0 || lowConstraint > 180 || 
        highConstraint < 0 || highConstraint > 180) return false;

    pinMode(controlPin, OUTPUT);

    this->controlPin = controlPin;
    this->lowConstraint = lowConstraint;
    this->highConstraint = highConstraint;
    
    write(0);
}

bool ServoMotor::isAttached() {
    return controlPin != -1;
}

void ServoMotor::write(int degrees) {

    if(!isAttached()) return;

    if(degrees > highConstraint) degrees = highConstraint;
    else if(degrees < lowConstraint) degrees = lowConstraint;

    writeMicroseconds(map(degrees, 0, FULL_RANGE, MIN_USECONDS, MAX_USECONDS));

    currAngle = degrees;
}

void ServoMotor::write() {
    write(currAngle);
}

void ServoMotor::writeMicroseconds(int highLength) {

    if(!isAttached()) return;

    static const int PULSE_DELAY = 21;

    if(millis() - pulseTimer > PULSE_DELAY) {
        
		noInterrupts();

        digitalWrite(controlPin, HIGH);
        delayMicroseconds(highLength);

        digitalWrite(controlPin, LOW);
        
        pulseTimer = millis();
		
		interrupts();
    }
}

int ServoMotor::read() {
    return currAngle;
}

void ServoMotor::enableCallback() {

    enableTimer();

    if(last_servo == MAX_SERVOS) return;
    for(int i = 0; i < last_servo; i++) if(callback_servos[i]->controlPin == controlPin) return;

    callback_servos[last_servo++] = this;
}

void ServoMotor::disableCallback() {

    int removedServoIndex = -1;

    for(int i = 0; i < last_servo; i++) {

        if (callback_servos[i]->controlPin == controlPin) {

            removedServoIndex = i;
            break;
        }
    }

    if(removedServoIndex != -1) {

        for(int i = removedServoIndex + 1; i < MAX_SERVOS; i++) callback_servos[i - 1] = callback_servos[i];
        last_servo--;
    }
}

void ServoMotor::enableSweep(int sweepStepSize, int start, int stop, SweepType sweepType) {
    
    enableCallback();

    sweepClockwise = start < stop;
    sweepStart = start;
    sweepStop = stop;

    if(sweepStart < lowConstraint) sweepStart = lowConstraint;
    if(sweepStart > highConstraint) sweepStart = highConstraint;
    if(sweepStop < lowConstraint) sweepStop = lowConstraint;
    if(sweepStop > highConstraint) sweepStop = highConstraint;

    this->sweepStepSize = sweepStepSize;
    this->sweepType = sweepType;
    
    currAngle = sweepStart;
    sweepMode = true;
    write();
}

void ServoMotor::disableSweep() {
    sweepMode = false;
}

bool ServoMotor::isSweeping() {
    return sweepMode;
}

void ServoMotor::sweep() {

    if(!isSweeping()) return;

    currAngle += sweepClockwise ? sweepStepSize : -sweepStepSize;

    if(sweepType == Reverse) {            
        
        ServoRange range = inRange(sweepStart, sweepStop);
        if(range == AboveRange) sweepClockwise = false;
        else if(range == BelowRange) sweepClockwise = true;
    }
    else if(sweepType == SkipReverse){

        if(inRange(sweepStart, sweepStop) != InRange) currAngle = sweepStart;
    }
    else if(sweepType == Single) {

        if(inRange(sweepStart, sweepStop) != InRange) {
                
            disableSweep();
            write(sweepStop);
        }
    }
}

ServoMotor::ServoRange ServoMotor::inRange(uint8_t low, uint8_t high) {

    if(low > high) {

        uint8_t temp = high;
        high = low;
        low = temp;
    }

    if(currAngle > high) return AboveRange;
    if(currAngle < low) return BelowRange;
    return InRange;
}

void ServoMotor::enableTimer() {

    // Checks if Timer2 is connected to the cpu clock. Connects it with a 1/1024 prescaler if not connected.
    if(TCCR2B & B00000111 == 0) TCCR2B |= B00000101;

    // Enables the overflow interrupt for Timer2;
    TIMSK2 |= (1 << TOIE2); 
}

ISR(TIMER2_OVF_vect) {

    static const int DELAY = 30;
    static volatile unsigned long timer = 0;

    if(millis() - timer > DELAY) {
        for(int i = 0; i < last_servo; i++) {
            callback_servos[i]->sweep();
            callback_servos[i]->write();
        }
        timer = millis();
    }
}