#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include "main.h"

/*
 * Number of intervals and points in one primitive trajectory.
 *
 * 50 intervals -> 51 points, including both endpoints.
 */
#define MOTION_INTERVALS    50
#define MOTION_POINTS       (MOTION_INTERVALS + 1)


/*
 * Generate a straight-line trajectory from the current Cartesian
 * position to the destination Cartesian position.
 *
 * Output:
 *   angles[0..50][0] = theta0 servo angle
 *   angles[0..50][1] = theta1 servo angle
 *   angles[0..50][2] = theta2 servo angle
 *
 * Returns:
 *   1 if all 51 trajectory points are valid.
 *   0 if any trajectory point is unreachable or outside
 *     the permitted joint-angle range.
 *
 * This function does NOT move any servos.
 */
uint8_t StraightLine(float cur_x,
                     float cur_y,
                     float cur_z,
                     float des_x,
                     float des_y,
                     float des_z,
                     float angles[MOTION_POINTS][3]);


/*
 * Generate an upward semicircular trajectory from the current
 * Cartesian position to the destination Cartesian position.
 *
 * Output:
 *   angles[0..50][0] = theta0 servo angle
 *   angles[0..50][1] = theta1 servo angle
 *   angles[0..50][2] = theta2 servo angle
 *
 * Returns:
 *   1 if all 51 trajectory points are valid.
 *   0 if any trajectory point is unreachable or outside
 *     the permitted joint-angle range.
 *
 * This function does NOT move any servos.
 */
uint8_t SemiCircle(float cur_x,
                   float cur_y,
                   float cur_z,
                   float des_x,
                   float des_y,
                   float des_z,
                   float angles[MOTION_POINTS][3]);

/*
 * Walk using an alternating-tripod gait.
 *
 * dir:
 *   Direction of travel in radians.
 *   0 = reference forward direction.
 *   Positive direction is clockwise when viewed from above.
 *
 * step_num:
 *   Number of full steps.
 *
 * step_size:
 *   Length of one full step in millimetres.
 *
 * step_time:
 *   Time taken for one full step in milliseconds.
 *
 * Returns:
 *   1 = motion successfully planned and executed
 *   0 = requested motion contains an invalid/unreachable position
 */
uint8_t Walk(float dir,
             uint16_t step_num,
             float step_size,
             uint32_t step_time);

/*
 * Move all six foot endpoints vertically by the same distance.
 *
 * elevation:
 *   Change in z coordinate in millimetres.
 *   Positive = +z
 *   Negative = -z
 *
 * total_time:
 *   Total time taken for the motion in milliseconds.
 *
 * Returns:
 *   1 = motion successfully planned and executed
 *   0 = requested motion contains an invalid/unreachable position
 */
uint8_t Vertical(float elevation,
                 uint32_t total_time);


/*
 * Expand or contract the hexagonal footprint of the robot.
 *
 * delta_x:
 *   Change in the local x coordinate of every foot endpoint.
 *   Positive = larger hexagonal footprint
 *   Negative = smaller hexagonal footprint
 *
 * total_time:
 *   Total time taken for the motion in milliseconds.
 *
 * Returns:
 *   1 = motion successfully planned and executed
 *   0 = requested motion contains an invalid/unreachable position
 */
uint8_t Spread(float delta_x,
               uint32_t total_time);

#endif /* INC_MOTION_H_ */

