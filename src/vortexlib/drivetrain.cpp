#include <mutex>
#include "vortexlib/drivetrain.h"

// ─────────────────────────────────────────────
//  CONSTRUCTOR
// ─────────────────────────────────────────────

/*
 * Initializes motor group references, sensor references, and computes
 * the wheel circumference and gear factor from the provided geometry.
 * All PID constants and motion parameters are taken from DriveConfig,
 * so tuning values stay centralized in main.cpp and never touch this file.
 *
 * gear_ratio: motor_teeth / wheel_teeth  (e.g. 60.0 / 36.0)
 * factor    : wheel revolutions per motor revolution (1.0 / gear_ratio)
 */
vortex::Drivetrain::Drivetrain(pros::MotorGroup& left_motors, 
                                pros::MotorGroup& right_motors,
                                pros::Rotation& left_rotation,
                                pros::Rotation& right_rotation,
                                pros::IMU& imu_sensor,
                                const Config& config
) 
    :
    left(left_motors),
    right(right_motors),
    rot_sensor_l(left_rotation),
    rot_sensor_r(right_rotation),
    imu(imu_sensor),
    circumference(get_wheel_perimeter(config.wheel_type)),
    factor(1.0 / config.gear_ratio),

    m_params {
        config.move_pid,
        config.turn_pid,
        config.params
    }
{
    current_pose = {0.0, 0.0, 0.0};
}

// ─────────────────────────────────────────────
//  CONFIGURATION
// ─────────────────────────────────────────────

/*
 * set_brake_mode(): applies the given brake mode to both motor groups.
 * Call this after any autonomous routine to return to driver-control feel
 * (e.g. coast or brake), since hold() may have changed it during auton.
 */
void vortex::Drivetrain::set_brake_mode(const pros::MotorBrake& mode) {

    left.set_brake_mode(mode);
    right.set_brake_mode(mode);
    previous_brake_mode = mode;
}

/*
 * set_debug(): activates debug flag member attribute
 * Motion methods print to console according to the state
 * of this flag. This is a handy functionality for PID tuning.
*/
void vortex::Drivetrain::set_debug() { debug = true; }

/*
 * set_reversed(): reverses both tracking encoders in software.
 * Use when physical mounting direction makes raw position readings negative
 * during forward motion. Must be called before the first update_odom() cycle.
 */
void vortex::Drivetrain::set_reversed() {
    rot_sensor_l.set_reversed(true);
    rot_sensor_r.set_reversed(true);
}

// ─────────────────────────────────────────────
//  TELEMETRY
// ─────────────────────────────────────────────

/*
 * get_distance(): returns the signed average displacement of both tracking
 * encoders since the last reset_positions() call, in centimeters.
 *
 * Signed: forward motion → positive, reverse → negative.
 * The VEX Rotation sensor returns centidegrees; 36000 = one full revolution.
 */
double vortex::Drivetrain::get_distance() const {

    double ticks_l = rot_sensor_l.get_position() / 36000.0;
    double ticks_r = rot_sensor_r.get_position() / 36000.0;
    return ((ticks_l + ticks_r) / 2.0) * circumference * factor;
}

/*
 * get_pose(): returns a copy of the current odometry position {x, y, heading}.
 * x and y are in centimeters from the origin set by set_pose().
 * heading mirrors imu.get_heading(): [0, 360), North = 0°, East = 90°.
 *
 * This is a snapshot — the background odom task keeps updating current_pose
 * concurrently; do not cache the result across multiple motion commands.
 */
vortex::Pose vortex::Drivetrain::get_pose() const {
    std::lock_guard<pros::Mutex> lock(const_cast<pros::Mutex&>(odom_mutex));
    return current_pose; 
}

/*
 * is_overheating(): returns true if any motor in either group exceeds 55 °C.
 * Iterates over each motor index because pros::MotorGroup does not expose a
 * direct max-temperature query. Conservative threshold for long matches.
 */
bool vortex::Drivetrain::is_overheating() const {

    const double TEMP_LIMIT = 55.0;

    for (int i = 0; i < (int)left.size(); i++) {
        if (left.get_temperature(i) >= TEMP_LIMIT) return true;
    }
    for (int i = 0; i < (int)right.size(); i++) {
        if (right.get_temperature(i) >= TEMP_LIMIT) return true;
    }
    return false;
}

/*
 * set_pose(): forces the odometry position to a known value.
 * Call at the start of each autonomous routine to set the field origin
 * relative to the robot's starting tile (adjust for alliance color as needed).
 * Does not reset encoder counts — call reset_positions() separately if needed.
 */
void vortex::Drivetrain::set_pose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    current_pose = pose;
}

// ─────────────────────────────────────────────
//  DRIVER CONTROL
// ─────────────────────────────────────────────

/*
 * arcade(): maps joystick axes to differential drive.
 *
 * The cubic mapping (v³ / 127²) preserves sign, compresses low-speed inputs
 * for fine control around center, and reaches full output at the joystick limit.
 * A ±5 deadband on both axes prevents motor chatter from stick drift.
 *
 * forward : left stick Y  [-127, 127]
 * turn    : right stick X [-127, 127]
 */
void vortex::Drivetrain::arcade(int forward, int turn) {

    if (std::abs(forward) < 5) forward = 0;
    if (std::abs(turn)   < 5) turn    = 0;

    left.move (forward + turn);
    right.move(forward - turn);
}

/*
 * hold(): applies active HOLD braking to both motor groups.
 *
 * Saves the current brake mode before switching so it can be restored
 * after the optional hold duration. This lets face_angle() and move_centimeters()
 * end with a firm stop during autonomous without permanently changing the
 * driver-control feel.
 *
 * duration_ms = 0 : hold indefinitely (autonomous use — driver resumes control)
 * duration_ms > 0 : hold for that many milliseconds, then restore previous mode
 *                   (driver-control use — gives a firm stop before returning input)
 */
void vortex::Drivetrain::hold(int duration_ms) {

    left.set_brake_mode(pros::MotorBrake::hold);
    right.set_brake_mode(pros::MotorBrake::hold);

    left.brake();
    right.brake();

    if (duration_ms > 0) {
        pros::delay(duration_ms);

        left.set_brake_mode(previous_brake_mode);
        right.set_brake_mode(previous_brake_mode);
    }
}

// ─────────────────────────────────────────────
//  AUTONOMOUS — PRIMITIVES
// ─────────────────────────────────────────────

/*
 * calibrate_imu(): resets and waits for the IMU sensor to complete calibration.
 * Polls imu.is_calibrating() every 10 ms with a 2-second safety timeout
 * (200 attempts × 10 ms) to prevent blocking if the sensor is disconnected
 * or malfunctioning. Must be called before any autonomous routine that relies
 * on imu.get_heading().
 */
void vortex::Drivetrain::calibrate_imu() {

    imu.reset();
    uint8_t attempt = 0;
    while (imu.is_calibrating() && attempt < 200) {
        pros::delay(10);
        attempt++;
    }
}

/*
 * reset_positions(): zeroes both tracking encoder positions.
 * Call before each move_centimeters() if you want relative distance tracking,
 * or rely on get_distance() with a local start_dist snapshot to avoid
 * resetting shared encoder state mid-routine.
 */
void vortex::Drivetrain::reset_positions() {
    rot_sensor_l.reset_position();
    rot_sensor_r.reset_position();
}

/*
 * swing_turn(): pivots the chassis around one stationary wheel.
 *
 * The pivot side is set to HOLD brake and immediately braked; the active side
 * drives with a PD controller referenced to imu.get_rotation() (cumulative,
 * not wrapping) to avoid heading discontinuities across 0°/360°.
 *
 * Side::LEFT  → left wheel holds,  right wheel drives
 * Side::RIGHT → right wheel holds, left wheel drives
 *
 * Useful for aligning to field walls or game elements without lateral drift.
 * 3-second safety timeout prevents stalling if the robot is physically blocked.
 */
void vortex::Drivetrain::swing_turn(int degrees, const Side& side, int max_vel) {
    // Calculate the target heading by adding the desired delta to the current angle.
    double target_heading = imu.get_heading() + degrees;

    double error      = normalize_angle(target_heading - imu.get_heading());
    double last_error = error;
    double integral   = 0.0;

    uint32_t start_time    = pros::millis();
    uint32_t settled_since = 0;

    // ── Tunable Thresholds (Inherited from the turn PID) ────────────────────
    constexpr double EXIT_ERROR     = 1.0;
    constexpr double SETTLE_TIME_MS = 120.0;
    constexpr double INTEGRAL_ZONE  = 15.0;
    constexpr double INTEGRAL_LIMIT = 40.0;
    constexpr double FRICTION_ZONE  = 8.0;

    // ── Configure the static pivot ──────────────────────────────────────────
    // Locking the pivot axis with HOLD active ensures the robot rotates precisely
    // on that side of the chassis.
    if (side == Side::LEFT) {
        left.set_brake_mode(pros::MotorBrake::hold);
        left.brake();
    } else {
        right.set_brake_mode(pros::MotorBrake::hold);
        right.brake();
    }

    // 3-second safety timeout (same as face_angle)
    while ((pros::millis() - start_time) < 3000) {

        double heading = imu.get_heading();
        error = normalize_angle(target_heading - heading);

        // ── Integral with anti-windup ──────────────────── ─────────────────────
        if (m_params.turn_pid.ki != 0.0 && std::abs(error) < INTEGRAL_ZONE) {
            integral += error;
            integral  = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            integral = 0.0;
        }

        // ── Derivative ─────────────────────────────────────────────────────────
        double derivative = error - last_error;

        // ── Output PID ────────────────────────────────────────────────────────
        double output =
            (error      * m_params.turn_pid.kp) +
            (integral   * m_params.turn_pid.ki) +
            (derivative * m_params.turn_pid.kd);

        // Limit max voltage
        output = std::clamp(output, static_cast<double>(-max_vel), static_cast<double>(max_vel));

        // ── Friction Compensation ─────────────────────────────────────────
        // Note: A swing turn requires overcoming more friction than a center pivot point.
        // If you notice it stalling, you could multiply min_turn by approximately 1.2.
        if (std::abs(error) > EXIT_ERROR &&
            std::abs(error) < FRICTION_ZONE &&
            std::abs(output) < m_params.params.min_turn) {

            output = (output >= 0.0) ? m_params.params.min_turn : -m_params.params.min_turn;
        }

        // ── Settlement Filter (Settle Timer) ─────────────────────────────
        if (std::abs(error) < EXIT_ERROR) {
            if (settled_since == 0) settled_since = pros::millis();
            if ((pros::millis() - settled_since) > SETTLE_TIME_MS) break;
        } else {
            settled_since = 0;
        }

        // ── Telemetry Debug ──────────────────────────────────────────────────
        if (debug) {
            printf(
                "[SWING] side: %s | error: %.2f | integral: %.2f | deriv: %.2f | out: %.2f\n",
                (side == Side::LEFT ? "LEFT" : "RIGHT"), error, integral, derivative, output
            );
        }

        // ── Dynamic Voltage Injection to the Active Side ──────────────────────
        // The side chosen as the pivot is NOT updated in the loop (.move),
        // it remains under the active brake command (.brake) given initially.
        if (side == Side::LEFT) {
            // If the pivot is on the left, the right handles all the torque.
            // Note the negative sign: a positive error requires the right side to
            //push back so the chassis rotates clockwise.
            right.move(-static_cast<int>(output));
        } else {
            // If the pivot is on the right, the left moves.
            left.move(static_cast<int>(output));
        }

        last_error = error;
        pros::delay(20);
    }

    // ── Exit behavior ─────────────────────────────────────────────────────────
    hold(150);
    left.move(0);
    right.move(0);
}

// ─────────────────────────────────────────────
//  ODOMETRY
// ─────────────────────────────────────────────

/*
 * update_odom(): integrates (x, y) position from encoder deltas and IMU heading.
 *
 * Must be called from a background task every ~10 ms:
 *
 *   pros::Task odom_task([&](){
 *       while(true) { drivetrain.update_odom(); pros::delay(10); }
 *   });
 *
 * Arc model with averaged heading: projects each displacement delta using the
 * midpoint angle between the previous and current heading. Compared to a
 * straight-line (start-of-cycle) model, this halves integration error on
 * curved paths without adding sensor calls.
 *
 * Wrap fix: the heading delta is normalized to [-π, π] before computing the
 * midpoint, preventing a catastrophic average when crossing the 0°/360° boundary
 * (e.g. 359° → 1° would otherwise average to 180° instead of 0°).
 *
 * Field convention: North = +Y, East = +X  (matches imu.get_heading() = 0° North).
 * Set the origin with set_pose() at the start of each autonomous routine.
 */
void vortex::Drivetrain::update_odom() {
    // 1. Safely stores the previous heading before updating it
    double prev_heading_rad;
    {
        std::lock_guard<pros::Mutex> lock(odom_mutex);
        prev_heading_rad = current_pose.heading * M_PI / 180.0;
    }

    // 2. Calculate the position deltas (in centimeters)
    double pos_l  = (rot_sensor_l.get_position() / 36000.0) * circumference * factor;
    double pos_r  = (rot_sensor_r.get_position() / 36000.0) * circumference * factor;

    double delta_dist = ((pos_l - odom_prev_l) + (pos_r - odom_prev_r)) / 2.0;
    odom_prev_l = pos_l;
    odom_prev_r = pos_r;

    double imu_heading = imu.get_heading();
    double current_heading_rad = imu_heading * M_PI / 180.0;

    // 3. Compute heading change and odometry integration
    double delta_heading = current_heading_rad - prev_heading_rad;
    if (delta_heading >  M_PI) delta_heading -= 2.0 * M_PI;
    if (delta_heading < -M_PI) delta_heading += 2.0 * M_PI;

    double avg_heading_rad = prev_heading_rad + delta_heading / 2.0;
    double delta_x = delta_dist * std::sin(avg_heading_rad);
    double delta_y = delta_dist * std::cos(avg_heading_rad);

    // 4. Update the pose atomically with the computed deltas
    {
        std::lock_guard<pros::Mutex> lock(odom_mutex);
        current_pose.heading = imu_heading;
        current_pose.x += delta_x;
        current_pose.y += delta_y;
    }
}

// ─────────────────────────────────────────────
//  AUTONOMOUS — HIGH LEVEL
// ─────────────────────────────────────────────

/*
 * face_angle(): rotates to an absolute field heading using a PID controller.
 *
 * Uses imu.get_heading() [0, 360) rather than get_rotation() (cumulative) so
 * target headings map directly to field directions (0° = North, 90° = East).
 * normalize_angle() ensures the robot always takes the shortest rotation path.
 *
 * Integral windup mitigation: accumulation is gated to |error| < 15°.
 * Outside that zone the integral resets, avoiding large accumulated values
 * during the approach that would cause overshoot on arrival.
 *
 * Friction compensation: if the PID output falls below config.params.min_turn,
 * it is clamped up to that threshold so static friction is always overcome.
 * Applied before max_vel clamping to preserve the effective output range.
 *
 * Exit behavior differs by context:
 *   autonomous  → hold() indefinitely (robot must not drift between commands)
 *   driver ctrl → hold(150 ms) then zero voltage (driver resumes smoothly)
 */
void vortex::Drivetrain::face_angle(double target_heading, int max_vel) {

    double error      = normalize_angle(target_heading - imu.get_heading());
    double last_error = error;
    double integral   = 0.0;

    uint32_t start_time   = pros::millis();
    uint32_t settled_since = 0;

    // ── Tuneable thresholds ───────────────────────────────────────────────────
    //
    //  EXIT_ERROR    : tolerance band to start the settle timer (degrees)
    //  SETTLE_TIME   : how long error must stay inside EXIT_ERROR before exit (ms)
    //  INTEGRAL_ZONE : gate — integral only accumulates below this error (degrees)
    //                  set above the expected steady-state error so ki kicks in
    //                  before the robot fully stalls
    //  INTEGRAL_LIMIT: symmetric clamp on the integral term to prevent windup
    //  FRICTION_ZONE : friction compensation activates when error is inside this
    //                  band (degrees). Must be LARGER than EXIT_ERROR to ensure
    //                  the robot maintains torque through final settling stage.
    //                  Empirical: set to ~5–8× EXIT_ERROR for stable behavior.
    //
    constexpr double EXIT_ERROR     = 1.0;
    constexpr double SETTLE_TIME_MS = 120.0;
    constexpr double INTEGRAL_ZONE  = 15.0;
    constexpr double INTEGRAL_LIMIT = 40.0;
    constexpr double FRICTION_ZONE  = 8.0;   // was MIN_TURN_ZONE = 3.0 → too small

    while ((pros::millis() - start_time) < 3000) {

        double heading = imu.get_heading();
        error = normalize_angle(target_heading - heading);

        // ── Integral with anti-windup ─────────────────────────────────────────
        //
        // Gate: only accumulate when ki is nonzero and error is small enough
        // that windup is not a concern. Reset when outside the zone so that a
        // large approach does not carry a pre-loaded integral into the fine stage.
        //
        if (m_params.turn_pid.ki != 0.0 && std::abs(error) < INTEGRAL_ZONE) {
            integral += error;
            integral  = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            integral = 0.0;
        }

        // ── Derivative ────────────────────────────────────────────────────────
        double derivative = error - last_error;

        // ── PID output ────────────────────────────────────────────────────────
        double output =
            (error      * m_params.turn_pid.kp) +
            (integral   * m_params.turn_pid.ki) +
            (derivative * m_params.turn_pid.kd);

        // ── max_vel clamp ─────────────────────────────────────────────────────
        output = std::clamp(
            output,
            static_cast<double>(-max_vel),
            static_cast<double>( max_vel)
        );

        // ── Friction compensation ─────────────────────────────────────────────
        //
        // Applied AFTER the max_vel clamp to respect user-requested velocity limits.
        // Activates when: error is in the fine-approach zone AND PID output is
        // below the minimum torque needed to overcome static friction.
        // Result: continuous motion through final degrees instead of stalling.
        //
        if (std::abs(error) > EXIT_ERROR &&
            std::abs(error) < FRICTION_ZONE &&
            std::abs(output) < m_params.params.min_turn) {

            output = (output >= 0.0) ? m_params.params.min_turn : -m_params.params.min_turn;
        }

        // ── Settle timer ──────────────────────────────────────────────────────
        //
        // Requires error to remain inside EXIT_ERROR for SETTLE_TIME_MS before
        // declaring success. Prevents false exit from transient zero-crossings
        // during high-frequency oscillations.
        //
        if (std::abs(error) < EXIT_ERROR) {

            if (settled_since == 0)
                settled_since = pros::millis();

            if ((pros::millis() - settled_since) > SETTLE_TIME_MS)
                break;

        } else {
            settled_since = 0;
        }

        // ── Debug ─────────────────────────────────────────────────────────────
        if (debug) {
            printf(
                "heading: %.2f | error: %.2f | integral: %.2f | deriv: %.2f | out: %.2f\n",
                heading, error, integral, derivative, output
            );
        }

        // ── Drive ─────────────────────────────────────────────────────────────
        left.move ( static_cast<int>(output));
        right.move(-static_cast<int>(output));

        last_error = error;
        pros::delay(20);
    }

    // ── Exit behavior ─────────────────────────────────────────────────────────
    // Apply firm HOLD brake to prevent drift, then zero motors and restore
    // the previous brake mode for seamless driver-control transition.
    hold(150);
    left.move(0);
    right.move(0);
}

/*
 * move_centimeters(): drives a signed distance in centimeters along the current heading.
 *
 * Uses a local start_dist snapshot so the method is non-destructive to global
 * encoder state and can be called multiple times in the same routine without
 * calling reset_positions() in between.
 *
 * PID: proportional + optional integral (gated to INTEGRAL_ZONE to prevent windup)
 * + derivative scaled by dt for physical units (cm/s rather than cm/cycle).
 *
 * Slew rate: asymmetric — deceleration step is 2.5× the acceleration step so the
 * robot brakes much faster than it ramps up, reducing overshoot at the target.
 *
 * Friction compensation: if PID output falls below config.params.min_move while
 * error is still outside the exit tolerance, output is clamped up to min_move.
 *
 * Heading correction: a proportional term on the IMU heading error steers the
 * chassis straight. A small deadband (ANGLE_DEADBAND) attenuates the correction
 * when nearly aligned to prevent oscillation at low angular error.
 *
 * Exit behavior mirrors face_angle() — hold() in autonomous, timed hold + zero
 * voltage in driver control.
 */
void vortex::Drivetrain::move_centimeters(int target_cm, int max_vel) {

    double start_dist = get_distance();
    double target_angle = imu.get_heading();
    double current_speed = 0.0;

    double integral   = 0.0;
    double last_error = static_cast<double>(target_cm);

    constexpr double dt = 0.02; // 20 ms loop step

    // ──────────────────────────────────────────────────────────────────────────
    //  Tuneable thresholds and internal limits
    // ──────────────────────────────────────────────────────────────────────────
    //
    //  EXIT_ERROR     : tolerance band to start the settle timer (cm)
    //  SETTLE_TIME_MS : how long error must stay inside EXIT_ERROR before exit (ms)
    //  ANGLE_DEADBAND : window to dampen tiny gyro noise and prevent hunting (degrees)
    //  INTEGRAL_ZONE  : gate for translational integral accumulation (cm); helps prevent
    //                   windup during large distance errors
    //  INTEGRAL_LIMIT : symmetric clamp on integral to cap maximum accumulated error
    //  DECEL_STEP     : slew rate multiplier for braking; 2.5× accel_st for fast stop
    //
    constexpr double EXIT_ERROR     = 1.0;
    constexpr double SETTLE_TIME_MS = 120.0;
    constexpr double ANGLE_DEADBAND  = 0.2;
    constexpr double INTEGRAL_ZONE   = 12.0;
    constexpr double INTEGRAL_LIMIT  = 40.0;
    
    const double DECEL_STEP = m_params.params.accel_st * 2.5;

    uint32_t start_time    = pros::millis();
    uint32_t settled_since = 0;

    // 5-second safety timeout
    while ((pros::millis() - start_time) < 5000) {

        // ── Distance calculations ─────────────────────────────────────────────
        double current_dist = get_distance() - start_dist;
        double dist_error   = target_cm - current_dist;

        // ── Settle timer (Exit condition) ─────────────────────────────────────
        //
        // Requires the error to stay inside EXIT_ERROR for SETTLE_TIME_MS before
        // breaking. Prevents a single zero-crossing sample from exiting early 
        // while the robot is still undergoing inertial deceleration or oscillation.
        //
        if (std::abs(dist_error) <= EXIT_ERROR) {
            if (settled_since == 0)
                settled_since = pros::millis();

            if ((pros::millis() - settled_since) > SETTLE_TIME_MS)
                break;
        } else {
            settled_since = 0;
        }

        // ── Integral with anti-windup ─────────────────────────────────────────
        if (m_params.move_pid.ki != 0.0 && std::abs(dist_error) < INTEGRAL_ZONE) {
            integral += dist_error * dt;
            integral  = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            integral = 0.0;
        }

        // ── Derivative ────────────────────────────────────────────────────────
        double derivative = (dist_error - last_error) / dt;

        // ── Translational PID calculation ─────────────────────────────────────
        double target_speed =
            (dist_error * m_params.move_pid.kp) +
            (integral   * m_params.move_pid.ki) +
            (derivative * m_params.move_pid.kd);

        // Maximum velocity clamp
        target_speed = std::clamp(
            target_speed, 
            static_cast<double>(-max_vel), 
            static_cast<double>( max_vel)
        );

        // ── Friction compensation ─────────────────────────────────────────────
        //
        // Applied AFTER the max_vel clamp to respect velocity limits.
        // Boosts output to min_move when PID falls below the threshold needed
        // to overcome static friction in the approach zone.
        //
        if (std::abs(dist_error) > EXIT_ERROR && std::abs(target_speed) < m_params.params.min_move) {
            target_speed = (target_speed >= 0.0) ? m_params.params.min_move : -m_params.params.min_move;
        }

        // ── Bidirectional slew rate ──────────────────────────────────────────
        //
        // Asymmetric acceleration/deceleration: DECEL_STEP is 2.5× faster to brake
        // hard and prevent overshoot when approaching the target distance.
        //
        double step = (std::abs(target_speed) < std::abs(current_speed)) ? DECEL_STEP : m_params.params.accel_st;
        double delta = target_speed - current_speed;
        
        delta = std::clamp(delta, -step, step);
        current_speed += delta;

        // ── Angular correction (Heading hold) ─────────────────────────────────
        //
        // Uses IMU to steer straight along the target heading. Applies a deadband
        // (scale × 0.3) when error is small to suppress sensor noise and oscillation.
        //
        double angle_error = normalize_angle(target_angle - imu.get_heading());

        if (std::abs(angle_error) < ANGLE_DEADBAND) {
            angle_error *= 0.3;
        }

        int steer_output = static_cast<int>(angle_error * m_params.params.angle_kp);

        // ── Final motor output layout ─────────────────────────────────────────
        int left_command  = static_cast<int>(current_speed) + steer_output;
        int right_command = static_cast<int>(current_speed) - steer_output;

        // ── Telemetry Debug ───────────────────────────────────────────────────
        if (debug) {
            printf(
                "dist: %.2f | err: %.2f | spd: %.2f | target: %.2f | head: %.2f | ang_err: %.2f | steer: %d\n",
                current_dist, dist_error, current_speed, target_speed, imu.get_heading(), angle_error, steer_output
            );
        }

        // ── Motor output with safety clamping ──────────────────────────────────
        left.move(std::clamp(left_command, -max_vel, max_vel));
        right.move(std::clamp(right_command, -max_vel, max_vel));

        last_error = dist_error;
        pros::delay(20);
    }

    // ── Exit behavior ─────────────────────────────────────────────────────────
    // Apply firm HOLD brake to prevent drift, then zero motors and restore
    // the previous brake mode for seamless driver-control transition.
    hold(150);
    left.move(0);
    right.move(0);
}

/*
 * drive_to_point(): navigates to a field coordinate (x, y) by turning then moving.
 *
 * Algorithm: Compute bearing with atan2(dx, dy) [North = 0°], face that angle,
 * then drive the Euclidean distance. Smart reverse: if target requires >90° turn,
 * negate distance and flip heading by 180° to drive backward instead — cheaper.
 *
 * Prerequisite: update_odom() running in background, set_pose() called at routine
 * start to establish field origin. Odometry-dependent — errors accumulate over time.
 *
 * Limitation: Two-phase (turn + straight). For smooth curved paths, consider
 * Pure Pursuit or similar trajectory-following algorithms.
 */
void vortex::Drivetrain::drive_to_point(double x, double y, int max_vel) {

    double dx = x - current_pose.x;
    double dy = y - current_pose.y;

    double target_heading = std::atan2(dx, dy) * 180.0 / M_PI;

    double distance = std::sqrt((dx * dx) + (dy * dy));

    double angle_error = normalize_angle(target_heading - imu.get_heading());

    // Intelligent reverse
    bool reverse = false;

    if (std::abs(angle_error) > 90.0) {

        reverse = true;
        distance = -distance;
        target_heading = normalize_angle(target_heading + 180.0);
    }

    face_angle(target_heading, max_vel);

    move_centimeters(static_cast<int>(distance), max_vel);
}
