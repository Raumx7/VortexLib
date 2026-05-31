#pragma once

namespace vortex {

    // ─────────────────────────────────────────
    //  Helper types
    // ─────────────────────────────────────────

    enum class Side : uint8_t { LEFT, RIGHT };   // Used in swing_turn() - methods

    enum class Wheel : uint8_t {   // vex wheels v5
        OMNI_275,
        OMNI_325,
        OMNI_400,
        OMNI_4125,
        OMNI_4175
    };

    struct Pose {
        double x       = 0.0;   // cm, horizontal axis (East)
        double y       = 0.0;   // cm, vertical axis   (North)
        double heading = 0.0;   // degrees [0, 360)
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

    // ─────────────────────────────────────────
    //  Configuration utilities
    // ─────────────────────────────────────────

    // Returns the selected wheel circumference in centimeters
    [[nodiscard]] inline constexpr double get_wheel_perimeter(const Wheel& wheel) {
        switch (wheel) {
            case Wheel::OMNI_275 : return 2.75 * 2.54 * M_PI;
            case Wheel::OMNI_325 : return 3.25 * 2.54 * M_PI;
            case Wheel::OMNI_400 : return 4.00 * 2.54 * M_PI;
            case Wheel::OMNI_4125 : return 4.125 * 2.54 * M_PI;
            case Wheel::OMNI_4175 : return 4.175 * 2.54 * M_PI;
        }
        return 0.0;
    }

    // ─────────────────────────────────────────
    //  Math utilities
    // ─────────────────────────────────────────

    // Wraps any angle into [-180, 180].
    [[nodiscard]] inline constexpr double normalize_angle(double angle) {
        while (angle >  180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

} // namespace vortex