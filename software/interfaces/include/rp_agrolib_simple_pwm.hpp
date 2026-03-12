#pragma once

#include <algorithm>
#include <cstdint>

#include "pico/stdlib.h"          /**< Includes the standard RP2040 library for GPIO and other hardware interactions */
#include "hardware/gpio.h"        /**< Includes the GPIO-specific functions for controlling the pins */
#include "hardware/pwm.h"         /**< Includes functions for PWM control */
#include "hardware/clocks.h"      /**< Includes clock control functions, needed for configuring the PWM frequency */

/**
 * @brief PWM output control class for the RP2040.
 * 
 * This class provides a simple interface for controlling a single PWM pin,
 * including methods for setting the duty cycle, frequency, alignment, and enabling/disabling the PWM signal.
 */
class PWMout {
public:
    /**
     * @brief Enum class for PWM alignment options.
     * 
     * Alignments available are:
     * - Edge: Standard edge-aligned PWM.
     * - PhaseCorrect: Phase-correct PWM, useful for motor control and signal generation.
     */
    enum class Align { Edge, PhaseCorrect };

    /**
     * @brief Constructs a PWM output for a specific pin.
     * 
     * Initializes the GPIO pin as a PWM output, sets the frequency, and initializes the duty cycle.
     * 
     * @param pin The GPIO pin to use for PWM output.
     * @param freq_hz The frequency of the PWM signal in Hz (default is 1000 Hz).
     * @param duty The initial duty cycle as a fraction (default is 0%).
     * @param enabled Whether the PWM is enabled immediately upon initialization (default is true).
     * @param align The PWM alignment type (default is Edge).
     */
    PWMout(uint pin,
           float freq_hz = 1000.0f,
           float duty = 0.0f,
           bool enabled = true,
           Align align = Align::Edge)
        : pin_(pin), slice_(pwm_gpio_to_slice_num(pin)), align_(align) {
        gpio_set_function(pin_, GPIO_FUNC_PWM);
        set_freq(freq_hz);
        set_duty(duty);
        enable(enabled);
    }

    /**
     * @brief Enables or disables the PWM signal.
     * 
     * Enables or disables the PWM signal for the current slice.
     * 
     * @param en Whether to enable the PWM signal (default is true).
     */
    void enable(bool en = true) {
        pwm_set_enabled(slice_, en);
        enabled_ = en;
    }

    /**
     * @brief Disables the PWM signal.
     */
    void disable() { enable(false); }

    /**
     * @brief Checks whether the PWM signal is currently enabled.
     * 
     * @return true if the PWM is enabled, false otherwise.
     */
    bool enabled() const { return enabled_; }

    /**
     * @brief Gets the GPIO pin associated with the PWM signal.
     * 
     * @return The GPIO pin number.
     */
    uint pin() const { return pin_; }

    /**
     * @brief Gets the slice number associated with the PWM signal.
     * 
     * @return The PWM slice number.
     */
    uint slice() const { return slice_; }

    /**
     * @brief Gets the PWM channel associated with the GPIO pin.
     * 
     * @return The PWM channel corresponding to the pin.
     */
    pwm_chan_t channel() const { return pwm_gpio_to_channel(pin_); }

    /**
     * @brief Sets the duty cycle of the PWM signal.
     * 
     * The duty cycle should be between 0.0 (off) and 100.0 (fully on).
     * This method internally converts the input value to a [0.0, 1.0] range before setting the duty.
     * 
     * @param duty The duty cycle as a percentage (0.0 to 100.0).
     */
    void set_duty(float duty) {
        duty_ = clamp01_(duty / 100.0f);  /**< Convert the duty cycle percentage (0.0 to 100.0) to the 0.0 to 1.0 range */
        pwm_set_gpio_level(pin_, duty_to_level_(duty_));
    }

    /**
     * @brief Gets the current duty cycle of the PWM signal.
     * 
     * @return The current duty cycle as a fraction (0.0 to 1.0).
     */
    float duty() const { return duty_; }

    /**
     * @brief Assignment operator to directly set the duty cycle.
     * 
     * This allows for syntax like:
     * 
     * @code
     * pwm = 0.5f;  // Set duty cycle to 50%
     * @endcode
     * 
     * @param duty The duty cycle as a fraction (0.0 to 1.0).
     * @return Reference to this PWM object.
     */
    PWMout &operator=(float duty) {
        set_duty(duty);
        return *this;
    }

    /**
     * @brief Sets the PWM level (raw value).
     * 
     * This method allows you to set the PWM signal level directly, bypassing the duty cycle.
     * The level is normalized based on the wrap value.
     * 
     * @param level The PWM level (0 to 65535).
     */
    void set_level(uint16_t level) {
        duty_ = (wrap_ == 0) ? 0.0f : (float)std::min<uint32_t>(level, wrap_) / (float)wrap_;
        pwm_set_gpio_level(pin_, std::min<uint32_t>(level, wrap_));
    }

    /**
     * @brief Gets the current PWM level.
     * 
     * @return The current PWM level (0 to 65535).
     */
    uint16_t level() const {
        return (uint16_t)std::min<uint32_t>(duty_to_level_(duty_), wrap_);
    }

    /**
     * @brief Sets the frequency of the PWM signal.
     * 
     * The frequency of the PWM signal is determined by the system clock and a clock divider.
     * If the frequency is non-positive, the PWM signal is disabled.
     * 
     * @param hz The desired frequency in Hz.
     */
    void set_freq(float hz) {
        if (hz <= 0.0f) {
            enable(false);
            freq_hz_ = 0.0f;
            return;
        }

        sys_hz_ = clock_get_hz(clk_sys);
        compute_div_wrap_(hz, 65535);

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, wrap_);
        pwm_config_set_clkdiv(&cfg, clkdiv_);
        pwm_config_set_phase_correct(&cfg, align_ == Align::PhaseCorrect);

        pwm_init(slice_, &cfg, enabled_);
        pwm_set_gpio_level(pin_, duty_to_level_(duty_));
        freq_hz_ = hz;
    }

    /**
     * @brief Gets the current frequency of the PWM signal.
     * 
     * @return The current frequency in Hz.
     */
    float freq() const { return freq_hz_; }

    /**
     * @brief Gets the current wrap value for the PWM cycle.
     * 
     * @return The current wrap value (maximum counter value for the PWM).
     */
    uint16_t wrap() const { return wrap_; }

    /**
     * @brief Gets the current clock divider used for the PWM signal.
     * 
     * @return The clock divider used for the PWM signal.
     */
    float clkdiv() const { return clkdiv_; }

    /**
     * @brief Sets the alignment mode of the PWM signal.
     * 
     * The alignment determines whether the PWM signal uses edge-aligned or phase-correct PWM.
     * 
     * @param a The desired alignment mode (Edge or PhaseCorrect).
     */
    void set_align(Align a) {
        if (a == align_) return;
        align_ = a;
        if (freq_hz_ > 0.0f) set_freq(freq_hz_);
    }

    /**
     * @brief Gets the current alignment mode of the PWM signal.
     * 
     * @return The current alignment mode (Edge or PhaseCorrect).
     */
    Align align() const { return align_; }

    
        /**
     * @brief Inverts the PWM signal.
     * 
     * This method inverts the PWM output, which means that when the duty cycle is high, the output will be low, and vice versa.
     * 
     * @param invert Whether to invert the PWM signal (true to invert, false to leave normal).
     */
    void set_inverted(bool invert = true) {
        pwm_set_invert(slice_, invert);  // Inverts the signal for the given slice (channel 0 or 1)
    }

private:
    /**
     * @brief Clamps the value to the range [0.0, 1.0].
     * 
     * Ensures that the duty cycle is within valid bounds.
     * 
     * @param x The value to clamp.
     * @return The clamped value.
     */
    static float clamp01_(float x) { return std::max(0.0f, std::min(1.0f, x)); }

    /**
     * @brief Converts a duty cycle to a corresponding PWM level (0 to 65535).
     * 
     * This is used to convert the duty cycle to a level that can be written to the PWM output.
     * 
     * @param d The duty cycle (0.0 to 1.0).
     * @return The corresponding PWM level (0 to 65535).
     */
    uint16_t duty_to_level_(float d) const {
        if (wrap_ == 0) return 0;
        uint32_t lvl = (uint32_t)((double)d * (double)(wrap_ + 1));
        if (lvl > wrap_) lvl = wrap_;
        return (uint16_t)lvl;
    }

    /**
     * @brief Computes the necessary clock divider and wrap value based on the target frequency.
     * 
     * This method calculates the clock divider and wrap value that ensures the desired frequency.
     * 
     * @param target_hz The desired PWM frequency in Hz.
     * @param max_wrap The maximum wrap value.
     */
    void compute_div_wrap_(float target_hz, uint16_t max_wrap) {
        const uint32_t factor = (align_ == Align::PhaseCorrect) ? 2u : 1u;
        const double denom_hz = (double)target_hz * (double)factor;

        uint32_t wrap_try = (uint32_t)((double)sys_hz_ / denom_hz) - 1u;
        if (wrap_try <= max_wrap) {
            wrap_ = (uint16_t)std::max<uint32_t>(1u, wrap_try);
            clkdiv_ = 1.0f;
            return;
        }

        wrap_ = max_wrap;
        double div = (double)sys_hz_ / (denom_hz * (double)(wrap_ + 1u));
        if (div < 1.0) div = 1.0;

        if (div > 256.0) {
            div = 256.0;
            uint32_t wrap2 = (uint32_t)((double)sys_hz_ / (denom_hz * div)) - 1u;
            wrap_ = (uint16_t)std::min<uint32_t>(std::max<uint32_t>(1u, wrap2), max_wrap);
        }

        clkdiv_ = (float)div;
    }

    uint pin_{0};       /**< GPIO pin number */
    uint slice_{0};     /**< PWM slice number associated with the pin */
    Align align_{Align::Edge}; /**< PWM alignment type (Edge or PhaseCorrect) */
    uint32_t sys_hz_{0}; /**< System clock frequency */
    uint16_t wrap_{1000}; /**< Wrap value (maximum counter value for PWM) */
    float clkdiv_{1.0f}; /**< Clock divider for PWM signal */
    float duty_{0.0f};   /**< Current duty cycle (0.0 - 1.0) */
    float freq_hz_{1000.0f}; /**< PWM frequency in Hz */
    bool enabled_{true}; /**< Whether PWM is enabled or not */
};