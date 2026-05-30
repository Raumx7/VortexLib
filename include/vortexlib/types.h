#pragma once

namespace vortex {

    // ─────────────────────────────────────────
    //  Helper types
    // ─────────────────────────────────────────

    enum class Side { LEFT, RIGHT };    // Used in swing_turn() - methods

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