#pragma once
#include "Marlin.h"

#ifndef GALVO_H
#define GALVO_H

// Galvo related global variables
extern const float min_step_size;
extern float step_size[2];
extern const float max_steps_per_unit;
extern unsigned const int steps;
extern unsigned const int points;


//Trying to keep everything stored in 16 bits, so separating offsets vs coordinates
//Only the offsets need negative values.
struct coord {
	unsigned int x;
	unsigned int y;
};

//Structure to hold an x/y offset per calibration point
struct offset {
	int x;
	int y;
};

//The scale of the full grid with respect to full DAC space (0x0000 to 0xFFFF)
extern float g_scale[2];

//The shift value for DAC space.  This will induce a tilt on an axis and used to compensate for a non vertical center point
extern unsigned int t_shift[2];

//The center of the full grid with respect to full DAC space (0x0000 to 0xFFFF)
extern unsigned int g_center[2];

//The calculated minimum DAC value
extern unsigned int g_min[2];

//The calculated maximum DAC value.  -1 is needed in order to make it an even value (hacky... may need some better math)
extern unsigned int g_max[2];

//The calculated total size of the printable space in DAC units
extern unsigned int g_size[2];

// The calculated steps per mm (unrelated to printing steps per unit)
// This is used to calculate z_size and e
extern float steps_per_mm[2];

//The distance between each calibration point
extern unsigned int cal_step_size[2];

//The distance between the print bed and last galvo mirror expressed in steps
extern unsigned int z_size[2];

//The max theta (per axis) of the galvo as calculated by the size of the print area
extern const float t0_max;;

//The max theta expressed in DAC steps
extern float t_max[2];

//The distance between the mirrors in DAC steps
extern const unsigned int e;

//Global galvo position. 
extern unsigned int g_position[2];

//Computes the calibration offset table
void compute_calibration_offsets();

//Applies the offset table to a set of coordinates
void apply_offset(struct coord * val);

// calculates the absolute galvo position based on all offsets and calibration points
void abs_galvo_position(struct coord * val, float x, float y);

// gets the grid index 
float get_x(int i);
float get_y(int i);

int select_x_index(float x);

int select_y_index(float y);
#endif