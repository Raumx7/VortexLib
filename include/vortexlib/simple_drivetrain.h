#pragma once
#include "api.h"
#include "types.h"

using opt_imu = std::optional<std::reference_wrapper<pros::IMU>>;

namespace vortex {

    // ─────────────────────────────────────────
    //  SimpleDrivetrain class
    // ─────────────────────────────────────────

    class SimpleDrivetrain {
    public:

        struct Config {
            PIDConstants move_pid;          // PID constants for move_centimeters()
            PIDConstants turn_pid;          // PID constants for face_angle()
            MoveParams   params;            // shared motion and rotation parameters
            Wheel        wheel_type;        // enum class Wheel
            double       gear_ratio = 1.0;  // (driven / drive)
            double       track_wd;          // track width (distance between wheels)
        };

    private:

        struct MotionParams {
            PIDConstants move_pid;
            PIDConstants turn_pid;
            MoveParams   params;
        };

        pros::Motor& left;                  // left motor reference
        pros::Motor& right;                 // right motor reference

        pros::Rotation&   rot_sensor_l;     // left tracking encoder reference
        pros::Rotation&   rot_sensor_r;     // right tracking encoder reference
        opt_imu imu;                        // optional inertial sensor

        double circumference;               // M_PI * wheel_diameter (cm)
        double factor;                      // 1.0 / gear_ratio (wheel_turns / motor_turns)
        double track_width;                 // track width (distance between wheels)

        Pose        current_pose;           // accumulated odometry position
        pros::Mutex   odom_mutex;           // synchronize access to the robot's position
        double odom_prev_l = 0.0;           // last recorded value — left sensor
        double odom_prev_r = 0.0;           // last recorded value — right sensor

        const MotionParams m_params;        // motion parameters
        bool debug = false;                 // debug flag used in motion functions (printf)

        pros::MotorBrake previous_brake_mode = pros::MotorBrake::coast;     // stores the previous brake mode

    public:

        // ── Constructors ──────────────────────
        SimpleDrivetrain(pros::Motor& left_motor, 
                            pros::Motor& right_motor,
                            pros::Rotation& left_rotation,
                            pros::Rotation& right_rotation,
                            pros::IMU& imu_sensor,
                            const Config& config
        );

        SimpleDrivetrain(pros::Motor& left_motor, 
                            pros::Motor& right_motor,
                            pros::Rotation& left_rotation,
                            pros::Rotation& right_rotation,
                            const Config& config
        );

        // ── Configuration setters ──────────────
        void set_brake_mode(const pros::MotorBrake& mode);
        void set_debug();
        void set_reversed();

        // ── Telemetry getters ──────────────────
        [[nodiscard]] double get_distance()    const;   // signed average distance since last reset (cm)
        [[nodiscard]] Pose   get_pose()        const;   // odometry position {x, y, heading}
        [[nodiscard]] double get_heading()     const;   // field heading (from IMU if available, else sensors)
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