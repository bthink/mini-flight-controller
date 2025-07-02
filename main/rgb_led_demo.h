#ifndef _RGB_LED_DEMO_H_
#define _RGB_LED_DEMO_H_

#include "rgb_led.h"

/**
 * @brief Demo showing all RGB LED capabilities
 * 
 * This demo cycles through different LED modes:
 * - Rainbow effect (smooth color transitions)
 * - Color blink (cycling through different colors)
 * - Breathing effect (blue LED fading in/out) 
 * - Flight status simulation (Green/Blue/Yellow pattern)
 */
void rgb_led_run_full_demo(void);

/**
 * @brief Simple blink demo with single color
 * 
 * @param color RGB color to blink
 * @param period_ms Blink period in milliseconds
 * @param duration_s Duration in seconds (0 = infinite)
 */
void rgb_led_simple_blink_demo(rgb_color_t color, uint32_t period_ms, uint32_t duration_s);

/**
 * @brief Show flight controller status patterns
 * 
 * Demonstrates typical flight controller LED patterns:
 * - Red: Error/Disarmed
 * - Yellow: Warning/Calibrating
 * - Blue: GPS Lock
 * - Green: Armed/Ready
 * - Rainbow: Celebration/Test mode
 */
void rgb_led_flight_status_demo(void);

#endif // _RGB_LED_DEMO_H_ 