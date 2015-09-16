#include "Marlin.h"
#include "mesh_bed_leveling.h"

#ifdef GALVO_CALIBRATION

class galvo_calibration : public mesh_bed_leveling {
public:
	typedef struct {
		double x, y;
	} cal_offset;
	cal_offset cal_offset_values[MESH_NUM_X_POINTS][MESH_NUM_Y_POINTS];

	galvo_calibration();

	void reset();

	void set_cal_value(int ix, int iy, float dx, float dy) { 
		cal_offset_values[iy][ix].x = dx;
		cal_offset_values[iy][ix].y = dy;
	}

};

extern galvo_calibration mbl;

#endif  // GALVO_CALIBRATION
