#ifndef _Servo_Motor_h
#define _Servo_Motor_h

#include "Arduino.h"

enum SweepType {Reverse, SkipReverse, Single};

class ServoMotor {
public:

    // DISCLAIMER: This library overrides the overflow interrupt of Timer2.

    /** 
     * Constructor:
     * Creates a ServoMotor object that can be associated with a servo motor (after specifying the control
     * pin by calling [ServoMotorObject].attach([controlPinNumber]) ).
     *  
     * !! OTHER METHODS WON'T WORK IF THE SERVO ISN'T ATTACHED !!
     */
    ServoMotor();
    ~ServoMotor();

    /**
     * attach():
     * ------------------------------------------------------------------------------------------------
     * Associates this ServoMotor object with a servo connected to pin [controlPin], returns false if
     * the object couldn't be attached to a servo (Usually because the pin number wasn't logical). 
     * Optional arguments for the upper and lower constraits of the individual servo rotation can be specified.
     */
    bool attach(int controlPin, int lowConstraint = 0, int highConstraint = 180);
    bool isAttached(); // Returns true if the ServoMotor object is attached to a servo.

    /**
     * write():
     * ------------------------------------------------------------------------------------------------
     * Tells the servo to rotate to the given position (in degrees from 0 to 180). If the new position
     * isn't specified it will tell the servo to rotate to the last given position. (This may be required
     * if callback isn't enabled and the servo failed to fully rotate to the desired position).
     */
    void write(int degrees);
    void write();

    /**
     * writeMicroseconds():
     * ------------------------------------------------------------------------------------------------
     * Sends a HIGH signal on the servo control pin for [highLength] microseconds. This will not update the
     * value returned by read().
     */
    void writeMicroseconds(int highLength);

    /**
     * read():
     * ------------------------------------------------------------------------------------------------
     * Returns the last specified rotation angle (using write()) for the servo.
     */
    int read();

    /**
     * enableCallback()/disableCallback():
     * ------------------------------------------------------------------------------------------------
     * Enables timer interrupts for this servo (this uses the overflow interrupt of Timer2), which will 
     * call the write() function repeatedly. disableCallback() disables this, however the Timer2 overflow 
     * interrupt will still be occupied.
     */
    void enableCallback();
    void disableCallback();
    
    /**
     * enableSweep()/disableSweep():
     * ------------------------------------------------------------------------------------------------
     * Calling this will enable sweep mode for the servo. There are 3 modes to choose from:
     * 1) Reverse sweep - The Servo will sweep from [start] to [stop] and then in reverse indefinitely.
     * 2) SkipReverse sweep - The Servo will do the same as Reverse sweep, but skip the reverse-ing part 
     * and restart from [start].
     * 3) Single sweep - The servo will sweep from [start] to [stop] just once.
     * 
     * This is a non-blocking operation, so you are free to do other things in the process. 
     * !! DISABLING THIS WILL NOT DISABLE CALLBACK MODE !! so you will need to do it manually if you want to.
     */
    void enableSweep(int sweepStepSize = 5, int start = 0, int stop = 180, SweepType sweepType = Reverse);
    void disableSweep();
    bool isSweeping();
    
    /**
     * sweep():
     * ------------------------------------------------------------------------------------------------
     * Performs one sweep step (if sweep mode is enabled). This method is called by the Interrupt Service
     * Routine. You will most likely not use this directly.
     */
    void sweep(); 

private:
    
    enum ServoRange {AboveRange, BelowRange, InRange};

    static const int MIN_USECONDS = 5e2, MAX_USECONDS = 24e2;
    static const int FULL_RANGE = 180;
    volatile unsigned long pulseTimer;

    uint8_t controlPin;
    uint8_t lowConstraint, highConstraint;

    volatile int currAngle;
    volatile bool sweepMode, sweepClockwise;
    SweepType sweepType;
    uint8_t sweepStepSize, sweepStart, sweepStop;

    void enableTimer();
    ServoRange inRange(uint8_t low, uint8_t high);

};


#endif