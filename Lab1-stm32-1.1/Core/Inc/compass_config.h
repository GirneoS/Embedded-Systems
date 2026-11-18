#ifndef COMPASS_CONFIG_H
#define COMPASS_CONFIG_H

/* Sensor XY is the horizontal plane. Mounting offset and handedness are
 * measured by the calibration wizard, not guessed in compile-time signs. */

#define COMPASS_SAMPLE_MS 40U
#define COMPASS_FRAME_MS 200U
#define COMPASS_STALE_MS 500U
#define COMPASS_RETRY_MS 2000U
#define COMPASS_MIN_HORIZONTAL_CUT 50.0f /* 0.50 microtesla */
#define COMPASS_CAL_TIMEOUT_MS 180000U

#endif
