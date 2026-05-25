#pragma once
#include "api.h"

namespace vortex {

    // ─────────────────────────────────────────
    //  Tipos auxiliares
    // ─────────────────────────────────────────

    enum class Side { LEFT, RIGHT };    // Uso en swing_turn()

    struct Pose {
        double x       = 0.0;   // cm, eje horizontal del campo (Este)
        double y       = 0.0;   // cm, eje vertical del campo   (Norte)
        double heading = 0.0;   // grados [0, 360) — convención imu.get_heading()
    };

    struct PIDConstants {
        double kp = 0.0;        // constante kp
        double ki = 0.0;        // constante ki
        double kd = 0.0;        // constante kd
    };

    struct MoveParams {
        double angle_kp;        // control angular en move_centimeters()
        double accel_st;        // aceleración en move_centimeters()
        double min_move;        // min. output en move_centimeters()
        double min_turn;        // min. output en face_angle()
    };

    struct DriveConfig {
        PIDConstants move_pid;  // constantes de control para move_centimeters()
        PIDConstants turn_pid;  // constantes de control para face_angle()
        MoveParams params;      // parámetros para funciones de movimiento y rotación
    };

    // ─────────────────────────────────────────
    //  Clase DriveTrain
    // ─────────────────────────────────────────

    class Drivetrain {
    private:
    
        pros::MotorGroup& left;             // referencia al grupo de motores izquierdo
        pros::MotorGroup& right;            // referencia al grupo de motores derecho

        pros::IMU&        imu;              // referencia a la imu
        pros::Rotation&   rot_sensor_l;     // referencia al encoder izquierdo
        pros::Rotation&   rot_sensor_r;     // referencia al encoder derecho

        double circumference;               // M_PI * wheel_diameter (cm)
        double factor;                      // 1.0 / gear_ratio (vueltas_rueda x vueltas_motor)

        Pose   current_pose;                // posición odométrica acumulada
        double odom_prev_l = 0.0;           // último valor leído — sensor izquierdo
        double odom_prev_r = 0.0;           // último valor leído — sensor derecho

        DriveConfig config;                 // parámetros de configuración

        pros::MotorBrake previous_brake_mode = pros::MotorBrake::coast;     // guarda pros::MotorBrake anterior

        // Lleva cualquier ángulo al rango [-180, 180].
        [[nodiscard]] static constexpr double normalize_angle(double angle) {
            while (angle >  180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        }

    public:

        // ── Constructor ───────────────────────
        Drivetrain(pros::MotorGroup& left_motors, 
                    pros::MotorGroup& right_motors,
                    pros::IMU& imu_sensor,
                    pros::Rotation& left_rotation,
                    pros::Rotation& right_rotation,
                    const DriveConfig& config,
                    double wheel_diameter, 
                    double gear_ratio = 1.0
        );

        // ── Configuración ─────────────────────
        void set_brake_mode(const pros::MotorBrake& mode);
        void set_reversed();                        

        // ── Telemetría ────────────────────────
        [[nodiscard]] double     get_distance() const;                  
        [[nodiscard]] Pose       get_pose() const;
        [[nodiscard]] bool       is_overheating() const;

        void set_pose(const Pose& pose);                      

        // ── Control manual ────────────────────
        void arcade(int forward, int turn);         
        void hold(int duration_ms = 0);                      

        // ── Autonomía: primitivos ─────────────
        void calibrate_imu();
        void reset_positions();
        void move_centimeters(int target_cm, int max_vel = 127);
        void swing_turn(int degrees, const Side& side, int max_vel = 80);

        // ── Odometría ─────────────────────────
        void update_odom();                      

        // ── Autonomía: alto nivel ─────────────
        void face_angle(double target_heading, int max_vel = 80);
        void drive_to_point(double x, double y, int max_vel = 127);
    };

} // namespace vortex
