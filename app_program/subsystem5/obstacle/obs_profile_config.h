#ifndef OBS_PROFILE_CONFIG_H
#define OBS_PROFILE_CONFIG_H

/* Original values are placeholders until verified on the robot. */
#define OBS_ANGLE_AHEAD_DEG  100U
#define OBS_FINE_MAX_SAMPLES 9U
#define OBS_MIN_RANGE_MM     20U
#define OBS_MAX_RANGE_MM     1000U
#define OBS_DIST_NONE        0xFFFU
#define OBS_BLOCK_MM         700U
#define OBS_SIDE_RANGE_MM    800U
#define OBS_CLUSTER_GAP_MM   60U
#define OBS_ROBOT_WIDTH_MM   100U
#define OBS_ROBOT_LENGTH_MM  240U
#define OBS_SAFETY_MARGIN_MM 20U
#define OBS_DEPTH_MIN_MM     100U
#define OBS_BEAM_TRIM_DEG    0.0f
#define OBS_OFFSET_MAX_MM    1023U
/* Object side. Bench (7 cm bottle 8 cm off-centre at 30 cm, 6 scans): fitted
 * range trend 11-23 mm towards the object in 5 scans, +3 in one; centred
 * bottles +/-4 mm; a centred flat file -9 mm. A weak signal: CENTRE is the
 * safe answer below the threshold. */
#define OBS_SIDE_TREND_MM    10U   /* |trend| needed to call a side when an edge is unseen */
#define OBS_SIDE_CENTRE_MM   20U   /* |centre x| within this = centre (edges seen) */

#endif
