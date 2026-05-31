#include <mutex>
#include "vortexlib/simple_drivetrain.h"

// ─────────────────────────────────────────────
//  CONSTRUCTORS
// ─────────────────────────────────────────────

/*
 * Constructor with IMU: initializes motor references, sensor references,
 * and computes wheel circumference and gear factor from configuration.
 * All PID constants and motion parameters come from Config.
 */
vortex::SimpleDrivetrain::SimpleDrivetrain(pros::Motor& left_motor,
                                            pros::Motor& right_motor,
                                            pros::Rotation& left_rotation,
                                            pros::Rotation& right_rotation,
                                            pros::IMU& imu_sensor,
                                            const Config& config
) 
    : 
    left(left_motor), 
    right(right_motor),
    rot_sensor_l(left_rotation),
    rot_sensor_r(right_rotation),
    imu(std::ref(imu_sensor)),
    circumference(get_wheel_perimeter(config.wheel_type)),
    factor(1.0 / config.gear_ratio),
    track_width(config.track_wd),
    
    m_params {
        config.move_pid, 
        config.turn_pid, 
        config.params
    }
{
    current_pose = {0.0, 0.0, 0.0};
}

/*
 * Constructor without IMU: similar to the IMU version but imu is nullopt.
 * Heading estimation will fall back to dead reckoning from rotation sensors.
 */
vortex::SimpleDrivetrain::SimpleDrivetrain(pros::Motor& left_motor,
                                            pros::Motor& right_motor,
                                            pros::Rotation& left_rotation,
                                            pros::Rotation& right_rotation,
                                            const Config& config
) 
    : 
    left(left_motor), 
    right(right_motor),
    rot_sensor_l(left_rotation),
    rot_sensor_r(right_rotation),
    imu(std::nullopt),
    circumference(get_wheel_perimeter(config.wheel_type)),
    factor(1.0 / config.gear_ratio),
    track_width(config.track_wd),
    m_params {
        config.move_pid, 
        config.turn_pid, 
        config.params}
{
    current_pose = {0.0, 0.0, 0.0};
}

// ─────────────────────────────────────────────
//  CONFIGURATION
// ─────────────────────────────────────────────

/*
 * set_brake_mode(): applies the given brake mode to both motors.
 * Call this after autonomous routines to restore driver-control feel.
 */
void vortex::SimpleDrivetrain::set_brake_mode(const pros::MotorBrake& mode) {
    left.set_brake_mode(mode);
    right.set_brake_mode(mode);
    previous_brake_mode = mode;
}

/*
 * set_debug(): activates debug flag for motion methods.
 * Useful for PID tuning and telemetry output.
 */
void vortex::SimpleDrivetrain::set_debug() { debug = true; }

/*
 * set_reversed(): reverses both tracking encoders in software.
 * Call before first update_odom() if physical mounting direction is inverted.
 */
void vortex::SimpleDrivetrain::set_reversed() {
    rot_sensor_l.set_reversed(true);
    rot_sensor_r.set_reversed(true);
}

// ─────────────────────────────────────────────
//  TELEMETRY
// ─────────────────────────────────────────────

/*
 * get_distance(): returns the signed average displacement of both tracking
 * encoders since the last reset_positions() call, in centimeters.
 * Signed: forward → positive, reverse → negative.
 */
double vortex::SimpleDrivetrain::get_distance() const {
    double ticks_l = rot_sensor_l.get_position() / 36000.0;
    double ticks_r = rot_sensor_r.get_position() / 36000.0;
    return ((ticks_l + ticks_r) / 2.0) * circumference * factor;
}

/*
 * get_pose(): returns a snapshot of the current odometry position {x, y, heading}.
 * x and y are in centimeters, heading is in degrees [0, 360).
 * Thread-safe: protected by odom_mutex.
 */
vortex::Pose vortex::SimpleDrivetrain::get_pose() const {
    std::lock_guard<pros::Mutex> lock(const_cast<pros::Mutex&>(odom_mutex));
    return current_pose;
}

/*
 * get_heading(): returns the current field heading in degrees.
 * If IMU is available: uses imu.get_heading().
 * If IMU is unavailable: calculates from rotation sensor deltas and track_width
 *   using dead reckoning (atan2 of differential wheel rotation).
 */
double vortex::SimpleDrivetrain::get_heading() const {
    if (imu.has_value()) {
        return imu.value().get().get_heading();
    } else {
        // Dead reckoning: estimate heading from encoder differential
        double pos_l  = (rot_sensor_l.get_position() / 36000.0) * circumference * factor;
        double pos_r  = (rot_sensor_r.get_position() / 36000.0) * circumference * factor;
        double delta_l_r = pos_r - pos_l;
        
        // Heading delta = atan2(lateral difference, track width)
        double heading_rad = std::atan2(delta_l_r, track_width);
        double heading_deg = heading_rad * 180.0 / M_PI;
        
        // Return normalized to [0, 360)
        while (heading_deg < 0.0)   heading_deg += 360.0;
        while (heading_deg >= 360.0) heading_deg -= 360.0;
        
        return heading_deg;
    }
}

/*
 * is_overheating(): returns true if either motor exceeds 55 °C.
 * Conservative threshold for long matches.
 */
bool vortex::SimpleDrivetrain::is_overheating() const {
    const double TEMP_LIMIT = 55.0;
    return (left.get_temperature() >= TEMP_LIMIT) || (right.get_temperature() >= TEMP_LIMIT);
}

/*
 * set_pose(): forces the odometry position to a known value.
 * Call at the start of each autonomous routine.
 * Thread-safe: protected by odom_mutex.
 */
void vortex::SimpleDrivetrain::set_pose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    current_pose = pose;
}

// ─────────────────────────────────────────────
//  DRIVER CONTROL
// ─────────────────────────────────────────────

/*
 * arcade(): maps joystick axes to differential drive.
 * A ±5 deadband on both axes prevents motor chatter from stick drift.
 *
 * forward : left stick Y  [-127, 127]
 * turn    : right stick X [-127, 127]
 */
void vortex::SimpleDrivetrain::arcade(int forward, int turn) {
    if (std::abs(forward) < 5) forward = 0;
    if (std::abs(turn)   < 5) turn    = 0;

    left.move (forward + turn);
    right.move(forward - turn);
}

/*
 * hold(): applies active HOLD braking to both motors.
 * Saves the current brake mode and restores it after optional duration.
 * duration_ms = 0 : hold indefinitely (autonomous use)
 * duration_ms > 0 : hold for that many milliseconds, then restore
 */
void vortex::SimpleDrivetrain::hold(int duration_ms) {
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
 * Polls imu.is_calibrating() every 10 ms with a 2-second safety timeout.
 * Only callable if IMU is present; does nothing if constructed without IMU.
 */
void vortex::SimpleDrivetrain::calibrate_imu() {
    if (!imu.has_value()) return;
    
    imu.value().get().reset();
    uint8_t attempt = 0;
    while (imu.value().get().is_calibrating() && attempt < 200) {
        pros::delay(10);
        attempt++;
    }
}

/*
 * reset_positions(): zeroes both tracking encoder positions.
 * Call before each move_centimeters() for relative distance tracking.
 */
void vortex::SimpleDrivetrain::reset_positions() {
    rot_sensor_l.reset_position();
    rot_sensor_r.reset_position();
}

/*
 * swing_turn(): pivots the chassis around one stationary wheel.
 * Uses get_heading() for feedback, which falls back to dead reckoning if no IMU.
 */
void vortex::SimpleDrivetrain::swing_turn(int degrees, const Side& side, int max_vel) {
    double target_heading = get_heading() + degrees;

    double error      = normalize_angle(target_heading - get_heading());
    double last_error = error;
    double integral   = 0.0;

    uint32_t start_time    = pros::millis();
    uint32_t settled_since = 0;

    constexpr double EXIT_ERROR     = 1.0;
    constexpr double SETTLE_TIME_MS = 120.0;
    constexpr double INTEGRAL_ZONE  = 15.0;
    constexpr double INTEGRAL_LIMIT = 40.0;
    constexpr double FRICTION_ZONE  = 8.0;

    // Configure the static pivot
    if (side == Side::LEFT) {
        left.set_brake_mode(pros::MotorBrake::hold);
        left.brake();
    } else {
        right.set_brake_mode(pros::MotorBrake::hold);
        right.brake();
    }

    while ((pros::millis() - start_time) < 3000) {
        double heading = get_heading();
        error = normalize_angle(target_heading - heading);

        if (m_params.turn_pid.ki != 0.0 && std::abs(error) < INTEGRAL_ZONE) {
            integral += error;
            integral  = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            integral = 0.0;
        }

        double derivative = error - last_error;

        double output =
            (error      * m_params.turn_pid.kp) +
            (integral   * m_params.turn_pid.ki) +
            (derivative * m_params.turn_pid.kd);

        output = std::clamp(output, static_cast<double>(-max_vel), static_cast<double>(max_vel));

        if (std::abs(error) > EXIT_ERROR &&
            std::abs(error) < FRICTION_ZONE &&
            std::abs(output) < m_params.params.min_turn) {
            output = (output >= 0.0) ? m_params.params.min_turn : -m_params.params.min_turn;
        }

        if (std::abs(error) < EXIT_ERROR) {
            if (settled_since == 0) settled_since = pros::millis();
            if ((pros::millis() - settled_since) > SETTLE_TIME_MS) break;
        } else {
            settled_since = 0;
        }

        if (debug) {
            printf(
                "[SWING] side: %s | error: %.2f | integral: %.2f | deriv: %.2f | out: %.2f\n",
                (side == Side::LEFT ? "LEFT" : "RIGHT"), error, integral, derivative, output
            );
        }

        if (side == Side::LEFT) {
            right.move(-static_cast<int>(output));
        } else {
            left.move(static_cast<int>(output));
        }

        last_error = error;
        pros::delay(20);
    }

    hold(150);
    left.move(0);
    right.move(0);
}

// ─────────────────────────────────────────────
//  ODOMETRY
// ─────────────────────────────────────────────

/*
 * update_odom(): integrates (x, y) position from encoder deltas and heading.
 * Must be called from a background task every ~10 ms.
 * Uses get_heading() for feedback (IMU if available, else dead reckoning).
 */
void vortex::SimpleDrivetrain::update_odom() {
    double prev_heading_rad;
    {
        std::lock_guard<pros::Mutex> lock(odom_mutex);
        prev_heading_rad = current_pose.heading * M_PI / 180.0;
    }

    double pos_l  = (rot_sensor_l.get_position() / 36000.0) * circumference * factor;
    double pos_r  = (rot_sensor_r.get_position() / 36000.0) * circumference * factor;

    double delta_dist = ((pos_l - odom_prev_l) + (pos_r - odom_prev_r)) / 2.0;
    odom_prev_l = pos_l;
    odom_prev_r = pos_r;

    double current_heading = get_heading();
    double current_heading_rad = current_heading * M_PI / 180.0;

    double delta_heading = current_heading_rad - prev_heading_rad;
    if (delta_heading >  M_PI) delta_heading -= 2.0 * M_PI;
    if (delta_heading < -M_PI) delta_heading += 2.0 * M_PI;

    double avg_heading_rad = prev_heading_rad + delta_heading / 2.0;
    double delta_x = delta_dist * std::sin(avg_heading_rad);
    double delta_y = delta_dist * std::cos(avg_heading_rad);

    {
        std::lock_guard<pros::Mutex> lock(odom_mutex);
        current_pose.heading = current_heading;
        current_pose.x += delta_x;
        current_pose.y += delta_y;
    }
}

// ─────────────────────────────────────────────
//  AUTONOMOUS — HIGH LEVEL
// ─────────────────────────────────────────────

/*
 * face_angle(): rotates to an absolute field heading using PID.
 * Uses get_heading() for feedback (IMU if available, else dead reckoning).
 * 3-second safety timeout prevents indefinite loops.
 */
void vortex::SimpleDrivetrain::face_angle(double target_heading, int max_vel) {
    double error      = normalize_angle(target_heading - get_heading());
    double last_error = error;
    double integral   = 0.0;

    uint32_t start_time   = pros::millis();
    uint32_t settled_since = 0;

    constexpr double EXIT_ERROR     = 1.0;
    constexpr double SETTLE_TIME_MS = 120.0;
    constexpr double INTEGRAL_ZONE  = 15.0;
    constexpr double INTEGRAL_LIMIT = 40.0;
    constexpr double FRICTION_ZONE  = 8.0;

    while ((pros::millis() - start_time) < 3000) {
        double heading = get_heading();
        error = normalize_angle(target_heading - heading);

        if (m_params.turn_pid.ki != 0.0 && std::abs(error) < INTEGRAL_ZONE) {
            integral += error;
            integral  = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            integral = 0.0;
        }

        double derivative = error - last_error;

        double output =
            (error      * m_params.turn_pid.kp) +
            (integral   * m_params.turn_pid.ki) +
            (derivative * m_params.turn_pid.kd);

        output = std::clamp(
            output,
            static_cast<double>(-max_vel),
            static_cast<double>( max_vel)
        );

        if (std::abs(error) > EXIT_ERROR &&
            std::abs(error) < FRICTION_ZONE &&
            std::abs(output) < m_params.params.min_turn) {
            output = (output >= 0.0) ? m_params.params.min_turn : -m_params.params.min_turn;
        }

        if (std::abs(error) < EXIT_ERROR) {
            if (settled_since == 0)
                settled_since = pros::millis();
            if ((pros::millis() - settled_since) > SETTLE_TIME_MS)
                break;
        } else {
            settled_since = 0;
        }

        if (debug) {
            printf(
                "heading: %.2f | error: %.2f | integral: %.2f | deriv: %.2f | out: %.2f\n",
                heading, error, integral, derivative, output
            );
        }

        left.move ( static_cast<int>(output));
        right.move(-static_cast<int>(output));

        last_error = error;
        pros::delay(20);
    }

    hold(150);
    left.move(0);
    right.move(0);
}

/*
 * move_centimeters(): drives a signed distance in centimeters with heading hold.
 * Non-destructive to encoder state — can be called multiple times without reset.
 * Uses get_heading() for heading correction (IMU if available, else dead reckoning).
 */
void vortex::SimpleDrivetrain::move_centimeters(int target_cm, int max_vel) {
    double start_dist = get_distance();
    double target_angle = get_heading();
    double current_speed = 0.0;

    double integral   = 0.0;
    double last_error = static_cast<double>(target_cm);

    constexpr double dt = 0.02;

    constexpr double EXIT_ERROR     = 1.0;
    constexpr double SETTLE_TIME_MS = 120.0;
    constexpr double ANGLE_DEADBAND  = 0.2;
    constexpr double INTEGRAL_ZONE   = 12.0;
    constexpr double INTEGRAL_LIMIT  = 40.0;
    
    const double DECEL_STEP = m_params.params.accel_st * 2.5;

    uint32_t start_time    = pros::millis();
    uint32_t settled_since = 0;

    while ((pros::millis() - start_time) < 5000) {
        double current_dist = get_distance() - start_dist;
        double dist_error   = target_cm - current_dist;

        if (std::abs(dist_error) <= EXIT_ERROR) {
            if (settled_since == 0)
                settled_since = pros::millis();
            if ((pros::millis() - settled_since) > SETTLE_TIME_MS)
                break;
        } else {
            settled_since = 0;
        }

        if (m_params.move_pid.ki != 0.0 && std::abs(dist_error) < INTEGRAL_ZONE) {
            integral += dist_error * dt;
            integral  = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            integral = 0.0;
        }

        double derivative = (dist_error - last_error) / dt;

        double target_speed =
            (dist_error * m_params.move_pid.kp) +
            (integral   * m_params.move_pid.ki) +
            (derivative * m_params.move_pid.kd);

        target_speed = std::clamp(
            target_speed, 
            static_cast<double>(-max_vel), 
            static_cast<double>( max_vel)
        );

        if (std::abs(dist_error) > EXIT_ERROR && std::abs(target_speed) < m_params.params.min_move) {
            target_speed = (target_speed >= 0.0) ? m_params.params.min_move : -m_params.params.min_move;
        }

        double step = (std::abs(target_speed) < std::abs(current_speed)) ? DECEL_STEP : m_params.params.accel_st;
        double delta = target_speed - current_speed;
        
        delta = std::clamp(delta, -step, step);
        current_speed += delta;

        double angle_error = normalize_angle(target_angle - get_heading());

        if (std::abs(angle_error) < ANGLE_DEADBAND) {
            angle_error *= 0.3;
        }

        int steer_output = static_cast<int>(angle_error * m_params.params.angle_kp);

        int left_command  = static_cast<int>(current_speed) + steer_output;
        int right_command = static_cast<int>(current_speed) - steer_output;

        if (debug) {
            printf(
                "dist: %.2f | err: %.2f | spd: %.2f | target: %.2f | head: %.2f | ang_err: %.2f | steer: %d\n",
                current_dist, dist_error, current_speed, target_speed, get_heading(), angle_error, steer_output
            );
        }

        left.move(std::clamp(left_command, -max_vel, max_vel));
        right.move(std::clamp(right_command, -max_vel, max_vel));

        last_error = dist_error;
        pros::delay(20);
    }

    hold(150);
    left.move(0);
    right.move(0);
}

/*
 * drive_to_point(): navigates to a field coordinate (x, y) by turning then moving.
 * Computes bearing with atan2(dx, dy) and intelligently reverses if turn > 90°.
 * Requires update_odom() running in background and set_pose() called at routine start.
 */
void vortex::SimpleDrivetrain::drive_to_point(double x, double y, int max_vel) {
    double dx = x - current_pose.x;
    double dy = y - current_pose.y;

    double target_heading = std::atan2(dx, dy) * 180.0 / M_PI;

    double distance = std::sqrt((dx * dx) + (dy * dy));

    double angle_error = normalize_angle(target_heading - get_heading());

    if (std::abs(angle_error) > 90.0) {
        distance = -distance;
        target_heading = normalize_angle(target_heading + 180.0);
    }

    face_angle(target_heading, max_vel);
    move_centimeters(static_cast<int>(distance), max_vel);
}
