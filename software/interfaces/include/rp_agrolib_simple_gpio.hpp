#pragma once

#include "pico/stdlib.h"

#include "hardware/gpio.h"

/**
 * @brief GPIO input wrapper for RP2040.
 *
 * Provides a minimal C++ abstraction over a single GPIO configured as input.
 * Supports optional internal pull configuration and boolean-style usage.
 */
class GPIOin {
public:
    /**
     * @brief Internal pull resistor configuration.
     */
    enum class Pull { None, Up, Down };

    /**
     * @brief Construct a GPIO input.
     *
     * Initializes the GPIO, sets direction to input, and applies pull configuration.
     *
     * @param pin  GPIO number (0–29 on RP2040).
     * @param pull Internal pull configuration (default: Pull::None).
     */
    GPIOin(uint pin, Pull pull = Pull::None) : pin_(pin), pull_(pull) {
        gpio_init(pin_);
        gpio_set_dir(pin_, GPIO_IN);
        apply_pull();
    }

    /**
     * @brief Read the current logic level of the pin.
     *
     * @return true if pin is high, false if low.
     */
    bool read() const { return gpio_get(pin_); }

    /**
     * @brief Get the GPIO pin number.
     */
    uint pin() const { return pin_; }

    /**
     * @brief Get the configured pull mode.
     */
    Pull pull() const { return pull_; }

    /**
     * @brief Boolean conversion operator.
     *
     * Allows usage such as:
     * @code
     * if(button) { ... }
     * @endcode
     */
    explicit operator bool() const { return read(); }

    /**
     * @brief Compare pin state with boolean value.
     */
    bool operator==(bool v) const { return read() == v; }

    /**
     * @brief Compare pin state with boolean value.
     */
    bool operator!=(bool v) const { return read() != v; }

private:
    /**
     * @brief Apply the configured pull resistor setting.
     */
    void apply_pull() {
        if (pull_ == Pull::Up) {
            gpio_pull_up(pin_);
        } else if (pull_ == Pull::Down) {
            gpio_pull_down(pin_);
        } else {
            gpio_disable_pulls(pin_);
        }
    }

    uint pin_;   ///< GPIO pin number.
    Pull pull_;  ///< Configured pull mode.
};

/**
 * @brief GPIO output wrapper for RP2040.
 *
 * Provides a minimal C++ abstraction over a single GPIO configured as output.
 * Supports write, readback, toggle, and boolean-style usage.
 */
class GPIOout {
public:
    /**
     * @brief Construct a GPIO output.
     *
     * Initializes the GPIO, sets direction to output, and writes initial value.
     *
     * @param pin     GPIO number.
     * @param initial Initial output state (default: false).
     */
    GPIOout(uint pin, bool initial = false) : pin_(pin) {
        gpio_init(pin_);
        gpio_set_dir(pin_, GPIO_OUT);
        write(initial);
    }

    /**
     * @brief Write logic level to the pin.
     *
     * @param v true = high, false = low.
     */
    void write(bool v) { gpio_put(pin_, v); }

    /**
     * @brief Read the current logic level of the pin.
     *
     * @return true if high, false if low.
     */
    bool read() const { return gpio_get(pin_); }

    /**
     * @brief Toggle the output state of the pin.
     */
    void toggle() { gpio_xor_mask(1u << pin_); }

    /**
     * @brief Assignment operator for writing output.
     *
     * Allows:
     * @code
     * led = true;
     * @endcode
     */
    GPIOout &operator=(bool v) {
        write(v);
        return *this;
    }

    /**
     * @brief Boolean conversion operator.
     */
    explicit operator bool() const { return read(); }

    /**
     * @brief Compare output state with boolean value.
     */
    bool operator==(bool v) const { return read() == v; }

    /**
     * @brief Compare output state with boolean value.
     */
    bool operator!=(bool v) const { return read() != v; }

    /**
     * @brief Get the GPIO pin number.
     */
    uint pin() const { return pin_; }

private:
    uint pin_;  ///< GPIO pin number.
};

/**
 * @brief Variadic GPIO input bus.
 *
 * Creates a compile-time defined input bus composed of multiple GPIO pins.
 * Bus bit 0 corresponds to the first template pin, bit 1 to the second, etc.
 *
 * @tparam Pins GPIO pin numbers forming the bus.
 */
template<uint... Pins>
class GPIOBusIn {
    static constexpr uint Width = sizeof...(Pins);

public:
    /**
     * @brief Construct input bus.
     *
     * Initializes all specified GPIOs as inputs.
     */
    GPIOBusIn() {
        for (uint p : {Pins...}) {
            gpio_init(p);
            gpio_set_dir(p, GPIO_IN);
        }
    }

    /**
     * @brief Read packed bus value.
     *
     * Reads all GPIOs using gpio_get_all() and packs them into a
     * contiguous integer where bit 0 corresponds to the first pin.
     *
     * @return Packed bus value.
     */
    uint32_t read() const {
        uint32_t gpio_state = gpio_get_all();
        uint32_t result = 0;
        uint bit = 0;

        for (uint p : {Pins...}) {
            if (gpio_state & (1u << p)) {
                result |= (1u << bit);
            }
            ++bit;
        }

        return result;
    }

    /**
     * @brief Implicit conversion to uint32_t.
     */
    operator uint32_t() const { return read(); }

    /**
     * @brief Compare bus value with integer.
     */
    bool operator==(uint32_t v) const { return read() == v; }

    /**
     * @brief Compare bus value with integer.
     */
    bool operator!=(uint32_t v) const { return read() != v; }

    /**
     * @brief Get bus width (number of pins).
     */
    static constexpr uint width() { return Width; }
};

/**
 * @brief Variadic GPIO output bus.
 *
 * Creates a compile-time defined output bus composed of multiple GPIO pins.
 * Bus bit 0 corresponds to the first template pin.
 *
 * @tparam Pins GPIO pin numbers forming the bus.
 */
template<uint... Pins>
class GPIOBusOut {
    static constexpr uint Width = sizeof...(Pins);

    /**
     * @brief Compute GPIO bitmask for selected pins.
     */
    static constexpr uint32_t make_mask() {
        uint32_t m = 0;
        for (uint p : {Pins...}) {
            m |= (1u << p);
        }
        return m;
    }

    static constexpr uint32_t mask = make_mask();

public:
    /**
     * @brief Construct output bus.
     *
     * Initializes all specified GPIOs as outputs and writes initial value.
     *
     * @param initial Initial packed bus value.
     */
    GPIOBusOut(uint32_t initial = 0) {
        for (uint p : {Pins...}) {
            gpio_init(p);
            gpio_set_dir(p, GPIO_OUT);
        }
        write(initial);
    }

    /**
     * @brief Write packed value to bus.
     *
     * Bit 0 of value corresponds to first template pin.
     *
     * @param value Packed bus value.
     */
    void write(uint32_t value) {
        uint32_t out = 0;
        uint bit = 0;
        for (uint p : {Pins...}) {
            if (value & (1u << bit)) {
                out |= (1u << p);
            }
            ++bit;
        }
        gpio_put_masked(mask, out);
    }

    /**
     * @brief Assignment operator for writing bus value.
     */
    GPIOBusOut &operator=(uint32_t value) {
        write(value);
        return *this;
    }

private:
};
