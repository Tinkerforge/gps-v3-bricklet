// This example is not self-contained.
// It requires usage of the example driver specific to your platform.
// See the HAL documentation.

#include "src/bindings/hal_common.h"
#include "src/bindings/bricklet_gps_v3.h"

void check(int rc, const char *msg);
void example_setup(TF_HAL *hal);
void example_loop(TF_HAL *hal);

static TF_GPSV3 gps;

void example_setup(TF_HAL *hal) {
	// Create device object
	check(tf_gps_v3_create(&gps, NULL, hal), "create device object");

	// Get current coordinates
	uint32_t latitude, longitude; char ns, ew;
	check(tf_gps_v3_get_coordinates(&gps, &latitude, &ns, &longitude,
	                                &ew), "get coordinates");

	tf_hal_printf("Latitude: %d 1/%d °\n", latitude, 1000000);
	tf_hal_printf("N/S: %c\n", ns);
	tf_hal_printf("Longitude: %d 1/%d °\n", longitude, 1000000);
	tf_hal_printf("E/W: %c\n", ew);
}

void example_loop(TF_HAL *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
