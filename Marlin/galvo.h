#pragma once
#ifndef GALVO_H
#define GALVO_H

// Galvo related global variables
const float min_step_size = X_MAX_POS / (float)0xFFFF;
unsigned const int steps = CAL_GRID_SIZE;
unsigned const int points = steps + 1;


//Trying to keep every stored in 16 bits, so separating offsets vs coordinates
//Only the offsets need negative values.
struct coord {
	unsigned int x;
	unsigned int y;
};

//Structure to hold an x/y offset per calibration point
struct offset {
	int x;
	int y;
} offsets[points][points];

//The scale of the full grid with respect to full DAC space (0x0000 to 0xFFFF)
float g_scale[2] = { 
					GALVO_X_SCALE, 
					GALVO_Y_SCALE 
					};

//The shift value for DAC space.  This will induce a tilt on an axis and used to compensate for a non vertical center point
unsigned int t_shift[2] = { 0, 0 };

//The center of the full grid with respect to full DAC space (0x0000 to 0xFFFF)
unsigned int g_center[2] = { 
					GALVO_CENTER + t_shift[X_AXIS], 
					GALVO_CENTER + t_shift[Y_AXIS]
					};

//The calculated minimum DAC value
unsigned int g_min[2] = { 
					(int)((float)g_center[X_AXIS] * (1 - g_scale[X_AXIS])),
					(int)((float)g_center[Y_AXIS] * (1 - g_scale[Y_AXIS]))
					};

//The calculated maximum DAC value.  -1 is needed in order to make it an even value (hacky... may need some better math)
unsigned int g_max[2] = { 
					(int)((float)g_center[X_AXIS] * (1 + g_scale[X_AXIS])) - 1,
					(int)((float)g_center[Y_AXIS] * (1 + g_scale[Y_AXIS])) - 1
					};

//The calculated total size of the printable space in DAC units
unsigned int g_size[2] = { 
					g_max[X_AXIS] - g_min[X_AXIS], 
					g_max[Y_AXIS] - g_min[Y_AXIS] 
					};
//The calculated steps per mm (unrelated to printing steps per unit)
int steps_per_mm = g_size[X_AXIS] / (int)X_MAX_LENGTH;

//The distance between each calibration point
unsigned int cal_step_size[2] = {
					g_size[X_AXIS] / steps,
					g_size[Y_AXIS] / steps
					};

//The distance between the print bed and last galvo mirror expressed in steps
unsigned int z_size[2] = {
					(unsigned int)((float)(Y_MAX_LENGTH / Z_MAX_LENGTH) * (float)g_size[X_AXIS]),
					(unsigned int)((float)(Y_MAX_LENGTH / Z_MAX_LENGTH) * (float)g_size[Y_AXIS])
					};

//The max theta (per axis) of the galvo as calculated by the size of the print area
const float t0_max = atan((X_MAX_LENGTH / 2) / Z_MAX_LENGTH);

//The max theta expressed in DAC steps
float t_max[2] = {
					(float)(g_size[X_AXIS] / 2.0) / t0_max,
					(float)(g_size[Y_AXIS] / 2.0) / t0_max
					};

//The distance between the mirrors in DAC steps
const int e = E_DISTANCE * steps_per_mm;

//Global galvo position. 
unsigned int g_position[2] = { g_min[X_AXIS], g_min[Y_AXIS] };

// calculates the absolute galvo position based on all offsets and calibration points
unsigned int abs_galvo_position(int axis, float pos) {
	unsigned int abs_pos = g_min[axis] + g_size[axis] * pos / max_pos[axis];
	//needs section here to handle calibration grid
	return abs_pos;
}

//Computes the calibration offset table
void compute_calibration_offsets() {
	float x_pos = (float)g_min[X_AXIS], y_pos = (float)g_min[Y_AXIS];
	float x_tmp, y_tmp;
	for (int j = 0; j < points; j++) {
		for (int i = 0; i < points; i++) {
			x_tmp = x_pos - (float)g_center[X_AXIS];
			y_tmp = y_pos - (float)g_center[Y_AXIS];
			y_tmp = atan2(y_tmp, z_size[Y_AXIS]) * t_max[Y_AXIS]; // compute y first as its designated the independent (second galvo) axis
			x_tmp = atan2(x_tmp, sqrt((y_tmp * y_tmp) + ((float)z_size[X_AXIS] * (float)z_size[X_AXIS])) + (float)e) * t_max[X_AXIS];  // compute x next as its dependent on the Y movement
			x_tmp += (float)g_center[X_AXIS] - x_pos;
			y_tmp += (float)g_center[Y_AXIS] - y_pos;
			offsets[i][j].x = (int)x_tmp;
			offsets[i][j].y = (int)y_tmp;
			/*
			Serial.print("(");
			Serial.print(offsets[i][j].x);
			Serial.print(", ");
			Serial.print(offsets[i][j].y);
			Serial.print(")");
			*/
			x_pos += cal_step_size[X_AXIS];
			if (x_pos > g_max[X_AXIS]) x_pos = g_min[X_AXIS];
		}
		y_pos += cal_step_size[Y_AXIS];
		if (y_pos > g_max[Y_AXIS]) y_pos = g_min[Y_AXIS];
		//Serial.println("");
	}
}

void apply_offset(struct coord * val) {
	// shifting all coordinate values by 4 (divide by 16) in order to stop overflowing 32 bit longs
	// This reduces bilinear interpolation resolution, but is necessary in order to not use floats.  
	// This is not perfect and will still overflow if low values are chosen for number of grid points.

	unsigned int sx = cal_step_size[X_AXIS] >> 4, sy = cal_step_size[Y_AXIS] >> 4;
	unsigned int x = val->x - g_min[X_AXIS];
	unsigned int y = val->y - g_min[Y_AXIS];
	int xi0 = x / cal_step_size[X_AXIS],
		yi0 = y / cal_step_size[Y_AXIS];
	int xi1, yi1;
	if (xi0 * cal_step_size[X_AXIS] >= g_size[X_AXIS]) {
		xi1 = xi0;
		xi0 = xi1 - 1;
	}
	else {
		xi1 = xi0 + 1;
	}
	if (yi0 * cal_step_size[Y_AXIS] >= g_size[Y_AXIS]) {
		yi1 = yi0;
		yi0 = yi1 - 1;
	}
	else {
		yi1 = yi0 + 1;
	}
	long x0 = xi0 * sx,
		y0 = yi0 * sy,
		x1 = xi1 * sx,
		y1 = yi1 * sy;
	struct offset q00, q01, q10, q11;
	long a0, a1, a2, a3, a4, afx, afy;
	//loads the values of the 4 calibration points around the point requested
	
		q00.x = offsets[xi0][yi0].x;
		q01.x = offsets[xi0][yi1].x;
		q10.x = offsets[xi1][yi0].x;
		q11.x = offsets[xi1][yi1].x;
	
		q00.y = offsets[xi0][yi0].y;
		q01.y = offsets[xi0][yi1].y;
		q10.y = offsets[xi1][yi0].y;
		q11.y = offsets[xi1][yi1].y;
	
	// Really ugly bilinear equation from wikipedia...
	// May not actually be fast after all
		a0 = (x1 - x) * (y1 - y);
		a1 = (x - x0) * (y1 - y);
		a2 = (x1 - x) * (y - y0);
		a3 = (x - x0) * (y - y0);
		a4 = (x1 - x0) * (y1 - y0);
		afx = (q00.x*a0 + q10.x*a1 + q01.x*a2 + q11.x*a3) / a4;
		afy = (q00.y*a0 + q10.y*a1 + q01.y*a2 + q11.y*a3) / a4;
		val->x += afx;
		val->y += afy;
}
#endif