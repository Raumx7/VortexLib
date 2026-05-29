#pragma once
#include "api.h"

namespace vortex {

    // ─────────────────────────────────────────
    //  Helper types
    // ─────────────────────────────────────────

    enum class Side { LEFT, RIGHT };    // Used in swing_turn()

    struct Pose {
        double x       = 0.0;   // cm, horizontal axis (East)
        double y       = 0.0;   // cm, vertical axis   (North)
        double heading = 0.0;   // degrees [0, 360) — imu.get_heading() convention
    };

    struct PIDConstants {
        double kp = 0.0;        // proportional gain
        double ki = 0.0;        // integral gain
        double kd = 0.0;        // derivative gain
    };

    struct MoveParams {
        double angle_kp;        // heading correction gain in move_centimeters()
        double accel_st;        // acceleration step in move_centimeters()
        double min_move;        // minimum output threshold in move_centimeters()
        double min_turn;        // minimum output threshold in face_angle()
    };

    struct DriveConfig {
        PIDConstants move_pid;  // PID constants for move_centimeters()
        PIDConstants turn_pid;  // PID constants for face_angle()
        MoveParams   params;    // shared motion and rotation parameters
    };

    // ─────────────────────────────────────────
    //  Drivetrain class
    // ─────────────────────────────────────────

    class Drivetrain {
    private:
    
        pros::MotorGroup& left;             // left motor group reference
        pros::MotorGroup& right;            // right motor group reference

        pros::Rotation&   rot_sensor_l;     // left tracking encoder reference
        pros::Rotation&   rot_sensor_r;     // right tracking encoder reference
        pros::IMU&        imu;              // inertial sensor reference

        double circumference;               // M_PI * wheel_diameter (cm)
        double factor;                      // 1.0 / gear_ratio (wheel_turns / motor_turns)

        Pose   current_pose;                // accumulated odometry position
        double odom_prev_l = 0.0;           // last recorded value — left sensor
        double odom_prev_r = 0.0;           // last recorded value — right sensor

        DriveConfig config;                 // motion configuration parameters
        bool debug = false;                 // debug flag used in motion functions (printf)

        pros::MotorBrake previous_brake_mode = pros::MotorBrake::coast;     // stores the previous brake mode

        // Wraps any angle into [-180, 180]. Static: does not access instance members.
        [[nodiscard]] static constexpr double normalize_angle(double angle) {
            while (angle >  180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        }

    public:

        // ── Constructor ───────────────────────
        Drivetrain(pros::MotorGroup& left_motors, 
                    pros::MotorGroup& right_motors,
                    pros::Rotation& left_rotation,
                    pros::Rotation& right_rotation,
                    pros::IMU& imu_sensor,
                    const DriveConfig& config,
                    double wheel_diameter, 
                    double gear_ratio = 1.0
        );

        // ── Configuration ─────────────────────
        void set_brake_mode(const pros::MotorBrake& mode);
        void set_debug();
        void set_reversed();                        

        // ── Telemetry ─────────────────────────
        [[nodiscard]] double get_distance()    const;   // signed average distance since last reset (cm)
        [[nodiscard]] Pose   get_pose()        const;   // odometry position {x, y, heading}
        [[nodiscard]] bool   is_overheating()  const;   // true if any motor exceeds 55 °C

        void set_pose(const Pose& pose);                // force a known position (use at auton start)

        // ── Driver control ────────────────────
        void arcade(int forward, int turn);             // single-stick arcade drive
        void hold(int duration_ms = 0);                 // HOLD brake; restores previous mode after duration_ms

        // ── Autonomous: primitives ────────────
        void calibrate_imu();
        void reset_positions();
        void move_centimeters(int target_cm, int max_vel = 127);
        void swing_turn(int degrees, const Side& side, int max_vel = 80);

        // ── Odometry ──────────────────────────
        void update_odom();                             // call from a background task every ~10 ms

        // ── Autonomous: high level ────────────
        void face_angle(double target_heading, int max_vel = 80);
        void drive_to_point(double x, double y, int max_vel = 127);
    };

} // namespace vortex