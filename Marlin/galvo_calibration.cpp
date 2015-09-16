#include "galvo_calibration.h"

#ifdef GALVO_CALIBRATION

galvo_calibration::galvo_calibration() { reset(); }

galvo_calibration mbl;

void galvo_calibration::reset() {
	//active = 1;
	for (int y = 0; y < MESH_NUM_Y_POINTS; y++)
		for (int x = 0; x < MESH_NUM_X_POINTS; x++)
			set_cal_value(x, y, 0, 0);
}

#endif  // MESH_BED_LEVELING
