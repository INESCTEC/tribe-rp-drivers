#include "../include/rp_agrolib_hamamatsu_xenon.h"

uint HamamatsuXenon::uniqueIDctr = 0;

HamamatsuXenon::HamamatsuXenon(uint triggerPin_, uint triggerFreq_)
    : triggerPin(triggerPin_), triggerFreq(triggerFreq_) {
    uniqueID = ++uniqueIDctr;

    gpio_init(triggerPin);
    gpio_set_dir(triggerPin, GPIO_OUT);

    mutex = true;  // Initialize to allow first trigger
    desiredNrTriggers = 0;
    currentNrTriggers = 0;
}

// Single Flash
void HamamatsuXenon::singleShot() {
    gpio_put(triggerPin, 1);
    busy_wait_at_least_cycles(1250);  // 10us in 125MHz
    gpio_put(triggerPin, 0);
}

void HamamatsuXenon::multiShot(uint nrShots) {
    if (mutex) {
        mutex = false;
        int64_t interval = 1000000 / triggerFreq;  // Time per flash in microseconds
        desiredNrTriggers = nrShots;
        currentNrTriggers = 0;

        // Set up a repeating timer that calls triggerFlash() every `interval` microseconds
        add_repeating_timer_us(-interval, triggerFlash, this, &timer);
    }
}


// Non-Blocking TimeFrame Shot
void HamamatsuXenon::timeFrameShot(uint time_ms) {
    if (mutex) {
        mutex = false;
        int64_t interval = 1000000 / triggerFreq;  // Time per flash in microseconds

        desiredNrTriggers = time_ms * (triggerFreq / 1000.0);
        currentNrTriggers = 0;

        add_repeating_timer_us(-interval, triggerFlash, this, &timer);
    }
}

// Static Timer Function to Trigger Flashes
bool HamamatsuXenon::triggerFlash(struct repeating_timer *t) {
    HamamatsuXenon *lamp = (HamamatsuXenon *) t->user_data;

    // Fire a single shot
    lamp->singleShot();

    lamp->setCurrentNrTriggers(lamp->getCurrentNrTriggers() + 1);

    // Stop the timer when the desired number of triggers is reached
    if (lamp->getCurrentNrTriggers() >= lamp->getDesiredNrTriggers()) {
        bool success = cancel_repeating_timer(lamp->getTimerAddr());
        lamp->setMutex(true);

        return false;  // Stops the timer
    }

    return true;  // Keep running until canceled
}

void HamamatsuXenon::setFrequency(uint triggerFreq_) {
    triggerFreq = triggerFreq_;
}

void HamamatsuXenon::setPin(uint triggerPin_) {
    triggerPin = triggerPin_;
}

uint HamamatsuXenon::getID() const {
    return uniqueID;
}

// 🔄 Mutex Getters & Setters
void HamamatsuXenon::setMutex(bool state) {
    mutex = state;
}
bool HamamatsuXenon::getMutex() const {
    return mutex;
}

// 🔄 desiredNrTriggers Getters & Setters
void HamamatsuXenon::setDesiredNrTriggers(uint n) {
    desiredNrTriggers = n;
}
uint HamamatsuXenon::getDesiredNrTriggers() const {
    return desiredNrTriggers;
}

// 🔄 currentNrTriggers Getters & Setters
void HamamatsuXenon::setCurrentNrTriggers(uint n) {
    currentNrTriggers = n;
}
uint HamamatsuXenon::getCurrentNrTriggers() const {
    return currentNrTriggers;
}

struct repeating_timer *HamamatsuXenon::getTimerAddr() {
    return &timer;
}