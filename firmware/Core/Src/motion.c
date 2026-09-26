#include "motion.h"
#include "servo.h"

#include <math.h>


/*
 * Hexapod leg geometry in millimetres.
 *
 * L0 = coxa
 * L1 = femur
 * L2 = tibia
 */
#define L0              50.0f
#define L1              75.0f
#define L2              110.0f

#define RAD_TO_DEG      (180.0f / M_PI)


/*
 * Default Cartesian foot position in each leg's local coordinate frame.
 * Units: mm
 */
#define DEFAULT_X    151.851578f
#define DEFAULT_Y      0.0f
#define DEFAULT_Z    -66.1263275f

static float phase_angles[6][MOTION_POINTS][3];
/*
 * Current Cartesian foot position of each leg in its own local frame.
 *
 * Index:
 *   0 = Leg A
 *   1 = Leg B
 *   2 = Leg C
 *   3 = Leg D
 *   4 = Leg E
 *   5 = Leg F
 *
 * These values represent the persistent current state of the robot.
 */
static float current_position[6][3] =
{
    {DEFAULT_X, DEFAULT_Y, DEFAULT_Z},   // A
    {DEFAULT_X, DEFAULT_Y, DEFAULT_Z},   // B
    {DEFAULT_X, DEFAULT_Y, DEFAULT_Z},   // C
    {DEFAULT_X, DEFAULT_Y, DEFAULT_Z},   // D
    {DEFAULT_X, DEFAULT_Y, DEFAULT_Z},   // E
    {DEFAULT_X, DEFAULT_Y, DEFAULT_Z}    // F
};

/*
 * Calculate the servo angles required to place one foot at
 * Cartesian position (x, y, z).
 *
 * Output:
 *   angles[0] = servo angle 0
 *   angles[1] = servo angle 1
 *   angles[2] = servo angle 2
 *
 * Returns:
 *   1 = valid position and all servo angles are within -90 to +90 deg
 *   0 = position is unreachable or at least one servo angle is invalid
 *
 * This function does NOT move any servos.
 */
static uint8_t InverseKinematics(float x,
                                 float y,
                                 float z,
                                 float angles[3])
{
    float r;
    float D;

    float theta0;
    float theta1;
    float theta2;

    float servo0;
    float servo1;
    float servo2;

    /*
     * Horizontal distance from the coxa rotation axis.
     */
    r = sqrtf((x * x) + (y * y));

    /*
     * Raw IK value for theta2.
     */
    D = (((r - L0) * (r - L0)) +
         (z * z) -
         (L1 * L1) -
         (L2 * L2))
        / (2.0f * L1 * L2);

    /*
     * If |D| > 1, no real theta2 solution exists.
     */
    if ((D < -1.0f) || (D > 1.0f))
    {
        return 0;
    }

    /*
     * Same IK branch used in V1.0.
     */
    theta0 = atan2f(y, x);

    theta2 = atan2f(-sqrtf(1.0f - (D * D)),
                    D);

    theta1 = atan2f(z, r - L0)
           - atan2f(L2 * sinf(theta2),
                    L1 + (L2 * cosf(theta2)));

    /*
     * Convert mathematical IK angles into the actual servo-angle
     * convention used by the physical leg.
     */
    servo0 = theta0 * RAD_TO_DEG;
    servo1 = (theta1 * RAD_TO_DEG) - 5.0f;
    servo2 = -(theta2 * RAD_TO_DEG) - 60.0f;

    /*
     * Servo-angle validity check.
     */
    if ((servo0 < -90.0f) || (servo0 > 90.0f) ||
        (servo1 < -90.0f) || (servo1 > 90.0f) ||
        (servo2 < -90.0f) || (servo2 > 90.0f))
    {
        return 0;
    }

    angles[0] = servo0;
    angles[1] = servo1;
    angles[2] = servo2;

    return 1;
}


/*
 * Generate a 51-point straight-line trajectory.
 *
 * The trajectory contains 50 equal Cartesian intervals between
 * the current position and destination position.
 *
 * This function calculates and validates the complete angle
 * trajectory but does NOT move any servos.
 */
uint8_t StraightLine(float cur_x,
                     float cur_y,
                     float cur_z,
                     float des_x,
                     float des_y,
                     float des_z,
                     float angles[MOTION_POINTS][3])
{
    for (uint16_t i = 0; i < MOTION_POINTS; i++)
    {
        float t = (float)i / (float)MOTION_INTERVALS;

        float x = cur_x + t * (des_x - cur_x);
        float y = cur_y + t * (des_y - cur_y);
        float z = cur_z + t * (des_z - cur_z);

        /*
         * Immediately terminate trajectory generation if any
         * Cartesian point produces invalid servo angles.
         */
        if (!InverseKinematics(x,
                               y,
                               z,
                               angles[i]))
        {
            return 0;
        }
    }

    return 1;
}

/*
 * Generate a 51-point upward semicircular trajectory.
 *
 * The line between the current position and destination position
 * forms the diameter of the semicircle.
 *
 * The semicircle lies in a vertical plane and is oriented so that
 * its midpoint rises in the +z direction.
 *
 * This function calculates and validates the complete angle
 * trajectory but does NOT move any servos.
 */
uint8_t SemiCircle(float cur_x,
                   float cur_y,
                   float cur_z,
                   float des_x,
                   float des_y,
                   float des_z,
                   float angles[MOTION_POINTS][3])
{
    /*
     * Vector from current position to destination position.
     */
    float dx = des_x - cur_x;
    float dy = des_y - cur_y;
    float dz = des_z - cur_z;

    /*
     * Length of the diameter.
     */
    float diameter = sqrtf((dx * dx) +
                           (dy * dy) +
                           (dz * dz));

    /*
     * A semicircle cannot be defined if the start and destination
     * positions are identical.
     */
    if (diameter <= 0.0f)
    {
        return 0;
    }

    /*
     * Centre and radius of the circle.
     */
    float center_x = (cur_x + des_x) / 2.0f;
    float center_y = (cur_y + des_y) / 2.0f;
    float center_z = (cur_z + des_z) / 2.0f;

    float radius = diameter / 2.0f;

    /*
     * Unit vector p points from the circle centre toward the
     * current position.
     */
    float px = (cur_x - center_x) / radius;
    float py = (cur_y - center_y) / radius;
    float pz = (cur_z - center_z) / radius;

    /*
     * Construct a second unit vector q perpendicular to p.
     *
     * q lies in the plane formed by p and the local z-axis,
     * and is chosen so that q has a positive z component.
     *
     * This makes the semicircle rise upward.
     */
    float horizontal = sqrtf((px * px) + (py * py));

    float qx;
    float qy;
    float qz;

    if (horizontal > 0.000001f)
    {
        qx = -(pz * px) / horizontal;
        qy = -(pz * py) / horizontal;
        qz = horizontal;
    }
    else
    {
        /*
         * Degenerate case:
         * the diameter is vertical, so there is no unique
         * vertical plane containing both the diameter and z-axis.
         *
         * Choose +x as the perpendicular direction.
         */
        qx = 1.0f;
        qy = 0.0f;
        qz = 0.0f;
    }

    /*
     * Generate 51 equally spaced angular samples over pi radians.
     *
     * phi = 0  -> current position
     * phi = pi -> destination position
     */
    for (uint16_t i = 0; i < MOTION_POINTS; i++)
    {
        float phi = M_PI *
                    ((float)i / (float)MOTION_INTERVALS);

        float cos_phi = cosf(phi);
        float sin_phi = sinf(phi);

        float x = center_x +
                  radius * ((px * cos_phi) +
                            (qx * sin_phi));

        float y = center_y +
                  radius * ((py * cos_phi) +
                            (qy * sin_phi));

        float z = center_z +
                  radius * ((pz * cos_phi) +
                            (qz * sin_phi));

        /*
         * Immediately terminate trajectory generation if any
         * Cartesian point produces invalid servo angles.
         */
        if (!InverseKinematics(x,
                               y,
                               z,
                               angles[i]))
        {
            return 0;
        }
    }

    return 1;
}

/*
 * Execute one synchronized 51-point trajectory phase
 * for all six legs.
 *
 * angles[leg][point][joint]
 *
 * leg:
 *   0 = A
 *   1 = B
 *   2 = C
 *   3 = D
 *   4 = E
 *   5 = F
 *
 * joint:
 *   0 = coxa
 *   1 = femur
 *   2 = tibia
 */
static void ExecutePhase(
    float angles[6][MOTION_POINTS][3],
    uint32_t phase_time)
{
    /*
     * 51 points contain 50 time intervals.
     */
    uint32_t interval_time =
        phase_time / MOTION_INTERVALS;

    for (uint16_t point = 0;
         point < MOTION_POINTS;
         point++)
    {
        /*
         * Command all 18 servos for this trajectory point.
         *
         * Leg A -> servos  0,  1,  2
         * Leg B -> servos  3,  4,  5
         * ...
         * Leg F -> servos 15, 16, 17
         */
        for (uint8_t leg = 0; leg < 6; leg++)
        {
            uint8_t first_servo = leg * 3;

            Servo_SetAngle(first_servo + 0,
                           angles[leg][point][0]);

            Servo_SetAngle(first_servo + 1,
                           angles[leg][point][1]);

            Servo_SetAngle(first_servo + 2,
                           angles[leg][point][2]);
        }

        /*
         * There is no need to delay after the final point.
         */
        if (point < MOTION_INTERVALS)
        {
            HAL_Delay(interval_time);
        }
    }
}

uint8_t Vertical(float elevation,
                 uint32_t total_time)
{
    /*
     * No movement requested.
     */
    if (elevation == 0.0f)
    {
        return 1;
    }

    /*
     * ------------------------------------------------------------
     * 1. GENERATE AND VALIDATE ALL SIX TRAJECTORIES
     * ------------------------------------------------------------
     *
     * For every leg:
     *
     *     (x, y, z)
     *
     * becomes
     *
     *     (x, y, z + elevation)
     *
     * x and y remain unchanged.
     *
     * All six trajectories are generated and validated BEFORE
     * any servo is moved.
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        float cur_x = current_position[leg][0];
        float cur_y = current_position[leg][1];
        float cur_z = current_position[leg][2];

        float des_x = cur_x;
        float des_y = cur_y;
        float des_z = cur_z + elevation;

        if (!StraightLine(cur_x,
                          cur_y,
                          cur_z,
                          des_x,
                          des_y,
                          des_z,
                          phase_angles[leg]))
        {
            /*
             * At least one leg cannot reach the requested position.
             *
             * Since no motion has yet occurred, immediately return
             * without moving any servos.
             */
            return 0;
        }
    }

    /*
     * ------------------------------------------------------------
     * 2. EXECUTE ALL SIX TRAJECTORIES SIMULTANEOUSLY
     * ------------------------------------------------------------
     */
    ExecutePhase(phase_angles, total_time);

    /*
     * ------------------------------------------------------------
     * 3. UPDATE PERSISTENT CARTESIAN STATE
     * ------------------------------------------------------------
     *
     * Only update current_position after successful execution.
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        current_position[leg][2] += elevation;
    }

    return 1;
}

uint8_t Spread(float delta_x,
               uint32_t total_time)
{
    /*
     * No movement requested.
     */
    if (delta_x == 0.0f)
    {
        return 1;
    }

    /*
     * ------------------------------------------------------------
     * 1. GENERATE AND VALIDATE ALL SIX TRAJECTORIES
     * ------------------------------------------------------------
     *
     * For every leg:
     *
     *     (x, y, z)
     *
     * becomes
     *
     *     (x + delta_x, y, z)
     *
     * y and z remain unchanged.
     *
     * Because each leg uses its own local coordinate frame and
     * local +x points radially outward, increasing x for all six
     * legs enlarges the regular hexagonal footprint.
     *
     * Decreasing x contracts the footprint.
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        float cur_x = current_position[leg][0];
        float cur_y = current_position[leg][1];
        float cur_z = current_position[leg][2];

        float des_x = cur_x + delta_x;
        float des_y = cur_y;
        float des_z = cur_z;

        if (!StraightLine(cur_x,
                          cur_y,
                          cur_z,
                          des_x,
                          des_y,
                          des_z,
                          phase_angles[leg]))
        {
            /*
             * At least one leg cannot reach the requested position.
             *
             * No servos have moved yet, so abort the entire motion.
             */
            return 0;
        }
    }

    /*
     * ------------------------------------------------------------
     * 2. EXECUTE ALL SIX TRAJECTORIES SIMULTANEOUSLY
     * ------------------------------------------------------------
     */
    ExecutePhase(phase_angles, total_time);

    /*
     * ------------------------------------------------------------
     * 3. UPDATE PERSISTENT CARTESIAN STATE
     * ------------------------------------------------------------
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        current_position[leg][0] += delta_x;
    }

    return 1;
}

uint8_t Walk(float dir,
             uint16_t step_num,
             float step_size,
             uint32_t step_time)
{
	if (step_num == 0){
		return 1;
	}
    /*
     * Fixed mounting/orientation angles of legs A-F.
     *
     * dir increases clockwise when viewed from above.
     */
    const float beta[6] =
    {
        M_PI / 6.0f,          // A:  30 deg
        M_PI / 2.0f,          // B:  90 deg
        5.0f * M_PI / 6.0f,   // C: 150 deg
        7.0f * M_PI / 6.0f,   // D: 210 deg
        3.0f * M_PI / 2.0f,   // E: 270 deg
        11.0f * M_PI / 6.0f   // F: 330 deg
    };

    /*
     * Critical Cartesian coordinates.
     *
     * C0 = position of each leg when Walk() begins
     * Cb = critical backward position
     * Cf = critical forward position
     */
    float C0[6][3];
    float Cb[6][3];
    float Cf[6][3];

    /*
     * ------------------------------------------------------------
     * 1. SAVE CURRENT POSITIONS AS C0
     * ------------------------------------------------------------
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        C0[leg][0] = current_position[leg][0];
        C0[leg][1] = current_position[leg][1];
        C0[leg][2] = current_position[leg][2];
    }


    /*
     * ------------------------------------------------------------
     * 2. CALCULATE Cb AND Cf
     * ------------------------------------------------------------
     *
     * gamma = beta - dir
     *
     * Cb = C0 - (step_size / 2) [cos(gamma), sin(gamma), 0]
     * Cf = C0 + (step_size / 2) [cos(gamma), sin(gamma), 0]
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        float gamma = beta[leg] - dir;

        float half_dx =
            (step_size / 2.0f) * cosf(gamma);

        float half_dy =
            (step_size / 2.0f) * sinf(gamma);

        Cb[leg][0] = C0[leg][0] - half_dx;
        Cb[leg][1] = C0[leg][1] - half_dy;
        Cb[leg][2] = C0[leg][2];

        Cf[leg][0] = C0[leg][0] + half_dx;
        Cf[leg][1] = C0[leg][1] + half_dy;
        Cf[leg][2] = C0[leg][2];
    }


    /*
     * ------------------------------------------------------------
     * 3. VALIDATE ALL UNIQUE TRAJECTORIES
     * ------------------------------------------------------------
     *
     * For each leg, four unique paths must be valid:
     *
     *   1. C0 <-> Cb : semicircle
     *   2. C0 <-> Cf : semicircle
     *   3. Cb <-> Cf : semicircle
     *   4. Cb <-> Cf : straight line
     *
     * Reversed paths do not need separate validation because they
     * contain the same Cartesian trajectory points in reverse order.
     */
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        /*
         * C0 -> Cb semicircle
         */
        if (!SemiCircle(C0[leg][0],
                        C0[leg][1],
                        C0[leg][2],
                        Cb[leg][0],
                        Cb[leg][1],
                        Cb[leg][2],
                        phase_angles[leg]))
        {
            return 0;
        }

        /*
         * C0 -> Cf semicircle
         */
        if (!SemiCircle(C0[leg][0],
                        C0[leg][1],
                        C0[leg][2],
                        Cf[leg][0],
                        Cf[leg][1],
                        Cf[leg][2],
						phase_angles[leg]))
        {
            return 0;
        }

        /*
         * Cb -> Cf semicircle
         */
        if (!SemiCircle(Cb[leg][0],
                        Cb[leg][1],
                        Cb[leg][2],
                        Cf[leg][0],
                        Cf[leg][1],
                        Cf[leg][2],
						phase_angles[leg]))
        {
            return 0;
        }

        /*
         * Cb -> Cf straight line
         */
        if (!StraightLine(Cb[leg][0],
                          Cb[leg][1],
                          Cb[leg][2],
                          Cf[leg][0],
                          Cf[leg][1],
                          Cf[leg][2],
						  phase_angles[leg]))
        {
            return 0;
        }
    }
    /*
     * ------------------------------------------------------------
     * 4. PREPARATION PHASE 1
     * ------------------------------------------------------------
     *
     * A, C, E:
     *     C0 -> Cb by semicircle
     *
     * B, D, F:
     *     remain stationary at C0
     */

    for (uint8_t leg = 0; leg < 6; leg++)
    {
        if ((leg == 0) ||
            (leg == 2) ||
            (leg == 4))
        {
            SemiCircle(C0[leg][0],
                       C0[leg][1],
                       C0[leg][2],
                       Cb[leg][0],
                       Cb[leg][1],
                       Cb[leg][2],
                       phase_angles[leg]);
        }
        else
        {
            StraightLine(C0[leg][0],
                         C0[leg][1],
                         C0[leg][2],
                         C0[leg][0],
                         C0[leg][1],
                         C0[leg][2],
                         phase_angles[leg]);
        }
    }

    ExecutePhase(phase_angles,
                 step_time / 2);

    /*
     * ------------------------------------------------------------
     * 5. PREPARATION PHASE 2
     * ------------------------------------------------------------
     *
     * A, C, E:
     *     remain stationary at Cb
     *
     * B, D, F:
     *     C0 -> Cf by semicircle
     */

    for (uint8_t leg = 0; leg < 6; leg++)
    {
        if ((leg == 1) ||
            (leg == 3) ||
            (leg == 5))
        {
            SemiCircle(C0[leg][0],
                       C0[leg][1],
                       C0[leg][2],
                       Cf[leg][0],
                       Cf[leg][1],
                       Cf[leg][2],
                       phase_angles[leg]);
        }
        else
        {
            StraightLine(Cb[leg][0],
                         Cb[leg][1],
                         Cb[leg][2],
                         Cb[leg][0],
                         Cb[leg][1],
                         Cb[leg][2],
                         phase_angles[leg]);
        }
    }

    ExecutePhase(phase_angles,
                 step_time / 2);

    /*
     * ------------------------------------------------------------
     * 6. ALTERNATING FULL STEPS
     * ------------------------------------------------------------
     */
    for (uint16_t step = 0; step < step_num; step++)
    {
        /*
         * Odd-numbered physical step: 1, 3, 5, ...
         *
         * A, C, E:
         *     Cb -> Cf by semicircle
         *
         * B, D, F:
         *     Cf -> Cb by straight line
         */
        if ((step % 2) == 0)
        {
            for (uint8_t leg = 0; leg < 6; leg++)
            {
                if ((leg == 0) ||
                    (leg == 2) ||
                    (leg == 4))
                {
                    SemiCircle(Cb[leg][0],
                               Cb[leg][1],
                               Cb[leg][2],
                               Cf[leg][0],
                               Cf[leg][1],
                               Cf[leg][2],
                               phase_angles[leg]);
                }
                else
                {
                    StraightLine(Cf[leg][0],
                                 Cf[leg][1],
                                 Cf[leg][2],
                                 Cb[leg][0],
                                 Cb[leg][1],
                                 Cb[leg][2],
                                 phase_angles[leg]);
                }
            }
        }

        /*
         * Even-numbered physical step: 2, 4, 6, ...
         *
         * A, C, E:
         *     Cf -> Cb by straight line
         *
         * B, D, F:
         *     Cb -> Cf by semicircle
         */
        else
        {
            for (uint8_t leg = 0; leg < 6; leg++)
            {
                if ((leg == 0) ||
                    (leg == 2) ||
                    (leg == 4))
                {
                    StraightLine(Cf[leg][0],
                                 Cf[leg][1],
                                 Cf[leg][2],
                                 Cb[leg][0],
                                 Cb[leg][1],
                                 Cb[leg][2],
                                 phase_angles[leg]);
                }
                else
                {
                    SemiCircle(Cb[leg][0],
                               Cb[leg][1],
                               Cb[leg][2],
                               Cf[leg][0],
                               Cf[leg][1],
                               Cf[leg][2],
                               phase_angles[leg]);
                }
            }
        }

        /*
         * All six leg trajectories are now ready.
         * Execute this full step simultaneously.
         */
        ExecutePhase(phase_angles, step_time);
    }

    /*
     * ------------------------------------------------------------
     * 7. RETURN TO C0
     * ------------------------------------------------------------
     *
     * The final tripod positions depend on whether step_num
     * is odd or even.
     */
    if ((step_num % 2) == 1)
    {
        /*
         * After an odd number of full steps:
         *
         * A, C, E are at Cf
         * B, D, F are at Cb
         *
         * Recovery phase 1:
         * A, C, E: Cf -> C0 by semicircle
         * B, D, F: remain stationary at Cb
         */
        for (uint8_t leg = 0; leg < 6; leg++)
        {
            if ((leg == 0) ||
                (leg == 2) ||
                (leg == 4))
            {
                SemiCircle(Cf[leg][0],
                           Cf[leg][1],
                           Cf[leg][2],
                           C0[leg][0],
                           C0[leg][1],
                           C0[leg][2],
                           phase_angles[leg]);
            }
            else
            {
                StraightLine(Cb[leg][0],
                             Cb[leg][1],
                             Cb[leg][2],
                             Cb[leg][0],
                             Cb[leg][1],
                             Cb[leg][2],
                             phase_angles[leg]);
            }
        }

        ExecutePhase(phase_angles,
                     step_time / 2);


        /*
         * Recovery phase 2:
         * A, C, E: remain stationary at C0
         * B, D, F: Cb -> C0 by semicircle
         */
        for (uint8_t leg = 0; leg < 6; leg++)
        {
            if ((leg == 1) ||
                (leg == 3) ||
                (leg == 5))
            {
                SemiCircle(Cb[leg][0],
                           Cb[leg][1],
                           Cb[leg][2],
                           C0[leg][0],
                           C0[leg][1],
                           C0[leg][2],
                           phase_angles[leg]);
            }
            else
            {
                StraightLine(C0[leg][0],
                             C0[leg][1],
                             C0[leg][2],
                             C0[leg][0],
                             C0[leg][1],
                             C0[leg][2],
                             phase_angles[leg]);
            }
        }

        ExecutePhase(phase_angles,
                     step_time / 2);
    }
    else
    {
        /*
         * After an even number of full steps:
         *
         * A, C, E are at Cb
         * B, D, F are at Cf
         *
         * Recovery phase 1:
         * B, D, F: Cf -> C0 by semicircle
         * A, C, E: remain stationary at Cb
         */
        for (uint8_t leg = 0; leg < 6; leg++)
        {
            if ((leg == 1) ||
                (leg == 3) ||
                (leg == 5))
            {
                SemiCircle(Cf[leg][0],
                           Cf[leg][1],
                           Cf[leg][2],
                           C0[leg][0],
                           C0[leg][1],
                           C0[leg][2],
                           phase_angles[leg]);
            }
            else
            {
                StraightLine(Cb[leg][0],
                             Cb[leg][1],
                             Cb[leg][2],
                             Cb[leg][0],
                             Cb[leg][1],
                             Cb[leg][2],
                             phase_angles[leg]);
            }
        }

        ExecutePhase(phase_angles,
                     step_time / 2);


        /*
         * Recovery phase 2:
         * B, D, F: remain stationary at C0
         * A, C, E: Cb -> C0 by semicircle
         */
        for (uint8_t leg = 0; leg < 6; leg++)
        {
            if ((leg == 0) ||
                (leg == 2) ||
                (leg == 4))
            {
                SemiCircle(Cb[leg][0],
                           Cb[leg][1],
                           Cb[leg][2],
                           C0[leg][0],
                           C0[leg][1],
                           C0[leg][2],
                           phase_angles[leg]);
            }
            else
            {
                StraightLine(C0[leg][0],
                             C0[leg][1],
                             C0[leg][2],
                             C0[leg][0],
                             C0[leg][1],
                             C0[leg][2],
                             phase_angles[leg]);
            }
        }

        ExecutePhase(phase_angles,
                     step_time / 2);
    }

    return 1;
}
