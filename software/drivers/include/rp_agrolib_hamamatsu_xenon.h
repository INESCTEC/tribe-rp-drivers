#pragma once
#include <stdio.h>

#include "pico/stdlib.h"


class HamamatsuXenon {

public:
    HamamatsuXenon(uint triggerPin_, uint triggerFreq_);

    void singleShot();
    void multiShot(uint nrShots);
    void timeFrameShot(uint time_ms);

    void setFrequency(uint triggerFreq_);
    void setPin(uint triggerPin_);

    uint getID() const;

    void setMutex(bool state);
    bool getMutex() const;

    void setDesiredNrTriggers(uint n);
    uint getDesiredNrTriggers() const;

    void setCurrentNrTriggers(uint n);
    uint getCurrentNrTriggers() const;
    struct repeating_timer *getTimerAddr();

private:
    uint triggerFreq;
    uint triggerPin;

    uint uniqueID;  // In case you create more than 1 object, it will create an identifier unique to
                    // it
    static uint uniqueIDctr;

    static bool triggerFlash(struct repeating_timer *t);
    struct repeating_timer timer;

    volatile uint desiredNrTriggers, currentNrTriggers;
    volatile bool mutex;
};