/*
  laser.cpp - mUVe laser control library for Arduino using 16 bit timers
  Copyright (c) 2013 Timothy Schmidt
  Copyright (c) 2014 Ante Vukorepa
  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Marlin.h"
#include "laser.h"
#include "planner.h"

laser_t laser;

void laser_init()
{
  pinMode(LASER_FIRING_PIN, OUTPUT);

  #if LASER_CONTROL == 3
    pinMode(LASER_POWER_PIN, OUTPUT);
    analogWrite(LASER_FIRING_PIN, 1);  // let Arduino setup do it's thing to the PWM pin  

    TCCR4B = 0x00;  // stop Timer4 clock for register updates
    TCCR4A = 0x82; // Clear OC4A on match, fast PWM mode, lower WGM4x=14
    ICR4 = labs(F_CPU / LASER_PWM); // clock cycles per PWM pulse
    OCR4A = labs(F_CPU / LASER_PWM) - 1; // ICR4 - 1 force immediate compare on next tick
    TCCR4B = 0x18 | 0x01; // upper WGM4x = 14, clock sel = prescaler, start running
  
    noInterrupts();
    TCCR4B &= 0xf8; // stop timer, OC4A may be active now
    TCNT4 = labs(F_CPU / LASER_PWM); // force immediate compare on next tick
    ICR4 = labs(F_CPU / LASER_PWM); // set new PWM period
    TCCR4B |= 0x01; // start the timer with proper prescaler value
    interrupts();
  #endif

  // Initialize state to sane defaults
  laser.intensity = 100.0;
  laser.ppm = 10;
  laser.duration = 1000;
  laser.status = LASER_OFF;
  laser.firing = LASER_OFF;
  #ifdef MUVE_Z_PEEL
    laser.peel_distance = 2.0;
    laser.peel_speed = 2.0;
    laser.peel_pause = 0.0;
  #endif // MUVE_Z_PEEL

}

void laser_pulse_init() {
  // Initialize the counters
  laser.micron_counter = 0;
  laser.time_counter = 0;
  // Length of one pulse
  laser.microns_per_pulse = (unsigned int) (1000./laser.ppm);
  // Length of one X step in microns
  laser.micron_inc_x = (unsigned int) (1000./axis_steps_per_unit[0]);
  // Length of one Y step in microns
  laser.micron_inc_y = (unsigned int) (1000./axis_steps_per_unit[1]);
  // Length of one X+Y step in microns
  laser.micron_inc_diagonal = (unsigned int) sqrt(sq(laser.micron_inc_x) + sq(laser.micron_inc_y));
  // Duration of one laser pulse in Timer1 ticks
  // One Timer1 tick @ 16 Mhz CPU = 1 / (2 Mhz) = 0.5 us; so ticks = duration / 0.5 us = duration * 2
  laser.pulse_ticks = laser.duration << 1;
  // CORRECTION AFTER MEASURING
  // laser.pulse_ticks = laser.duration;
}

#if LASER_CONTROL == 1
unsigned long calc_laser_intensity(float intensity) {
  if (intensity > 100.0) intensity = 100.0; // restrict intensity between 0 and 255
  if (intensity < 0.0) intensity = 0.0;
  return labs(intensity*2.55);
}
#else
unsigned long calc_laser_intensity(float intensity) {
  if (intensity > 100.0) intensity = 100.0; // restrict intensity between 0 and 255
  if (intensity < 0.0) intensity = 0.0;
  return labs((intensity / 100.0)*(F_CPU / LASER_PWM));
}
#endif
