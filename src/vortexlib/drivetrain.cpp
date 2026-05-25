#include "vortexlib/drivetrain.h"

// ─────────────────────────────────────────────
//  CONSTRUCTOR
// ─────────────────────────────────────────────

vortex::Drivetrain::Drivetrain(pros::MotorGroup& left_motors, 
                                pros::MotorGroup& right_motors,
                                pros::IMU& imu_sensor,
                                pros::Rotation& left_rotation,
                                pros::Rotation& right_rotation,
                                const DriveConfig& config, 
                                double wheel_diameter, 
                                double gear_ratio
) 
    :
    left(left_motors),
    right(right_motors),
    imu(imu_sensor),
    rot_sensor_l(left_rotation),
    rot_sensor_r(right_rotation)

{
    this->config = config;
    
    circumference = M_PI * wheel_diameter;
    factor        = 1.0 / gear_ratio;

    current_pose  = {
        .x = 0.0, 
        .y = 0.0, 
        .heading = 0.0
    };
}

// ─────────────────────────────────────────────
//  CONFIGURACIÓN
// ─────────────────────────────────────────────

void vortex::Drivetrain::set_brake_mode(const pros::MotorBrake& mode) {
    left.set_brake_mode(mode);
    right.set_brake_mode(mode);
}

void vortex::Drivetrain::set_reversed() {
    rot_sensor_l.set_reversed(true);
    rot_sensor_r.set_reversed(true);
}

// ─────────────────────────────────────────────
//  TELEMETRÍA
// ─────────────────────────────────────────────


// get_distance(): promedio FIRMADO de ambos sensores.
double vortex::Drivetrain::get_distance() const {

    // get_position() devuelve centi-grados; 36000 = una vuelta completa
    double ticks_l = rot_sensor_l.get_position() / 36000.0;
    double ticks_r = rot_sensor_r.get_position() / 36000.0;
    return ((ticks_l + ticks_r) / 2.0) * circumference * factor;
}

vortex::Pose vortex::Drivetrain::get_pose() const {
    return current_pose;
}

// is_overheating(): revisa temperatura de todos los motores, true se se excede 55°
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

void vortex::Drivetrain::set_pose(const Pose& pose) {
    current_pose = pose;
}

// ─────────────────────────────────────────────
//  CONTROL MANUAL
// ─────────────────────────────────────────────

void vortex::Drivetrain::arcade(int forward, int turn) {
    if (std::abs(forward) < 5) forward = 0;
    if (std::abs(turn)   < 5) turn    = 0;

    // Curva cúbica: mayor precisión en movimientos finos
    int exp_forward = (forward * forward * forward) / (127 * 127);
    int exp_turn    = (turn    * turn    * turn)    / (127 * 127);

    left.move (exp_forward + exp_turn);
    right.move(exp_forward - exp_turn);
}

/*
 * hold(): aplica HOLD activo en el lugar.
 * Llama a brake() con el modo ya seteado; si quieres garantizar HOLD
 * independientemente del modo global, fuerza el cambio aquí.
 */
void vortex::Drivetrain::hold(int duration_ms) {

    previous_brake_mode = left.get_brake_mode(); 

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
//  AUTONOMÍA — PRIMITIVOS
// ─────────────────────────────────────────────

void vortex::Drivetrain::calibrate_imu() {

    imu.reset();
    uint8_t attempt = 0;
    while (imu.is_calibrating() && attempt < 200) {
        pros::delay(10);
        attempt++;
    }
}

void vortex::Drivetrain::reset_positions() {

    rot_sensor_l.reset_position();
    rot_sensor_r.reset_position();
}

// ── swing_turn ───────────────────────────────
/*
 * Gira pivotando sobre un lado: el lado estático frena (HOLD),
 * el lado activo empuja. Muy útil para alinearse a paredes/estructuras
 * sin desplazar el robot lateralmente.
 *
 * Side::LEFT  → pivota sobre rueda izquierda (solo derecha mueve)
 * Side::RIGHT → pivota sobre rueda derecha   (solo izquierda mueve)
 */
void vortex::Drivetrain::swing_turn(int degrees, const Side& side, int max_vel) {

    double target     = imu.get_rotation() + degrees;
    double error      = degrees;
    double last_error = degrees;

    const double kp = 1.2;
    const double kd = 0.5;

    // El lado pivote frena; el otro lado propulsa
    if (side == Side::LEFT) {
        left.set_brake_mode(pros::MotorBrake::hold);
        left.brake();
    } else {
        right.set_brake_mode(pros::MotorBrake::hold);
        right.brake();
    }

    uint32_t start_time = pros::millis();

    while (std::abs(error) > 1.0 && (pros::millis() - start_time < 3000)) {

        error = target - imu.get_rotation();
        double derivative = error - last_error;

        int voltage = static_cast<int>((error * kp) + (derivative * kd));
        voltage = (voltage >  max_vel) ?  max_vel : voltage;
        voltage = (voltage < -max_vel) ? -max_vel : voltage;

        if (side == Side::LEFT) {
            // Rueda izquierda quieta; derecha empuja en dirección opuesta
            right.move(-voltage);
        } else {
            left.move(voltage);
        }

        last_error = error;
        pros::delay(20);
    }

    left.brake();
    right.brake();
}

// ─────────────────────────────────────────────
//  ODOMETRÍA
// ─────────────────────────────────────────────

/*
 * update_odom(): integra posición (x, y) usando heading de la IMU
 * y desplazamiento diferencial de los sensores de rotación.
 *
 * Debe llamarse desde una tarea de fondo cada ~10 ms:
 *
 *   pros::Task odom_task([&](){
 *       while(true) { drivetrain.update_odom(); pros::delay(10); }
 *   });
 *
 * Modelo de arco con ángulo promedio: en vez de proyectar el delta
 * con el heading del inicio del ciclo, usa el punto medio entre el
 * heading anterior y el actual. Reduce el error de integración en
 * trayectorias curvas hasta en un 50% respecto al modelo lineal.
 *
 * Fix de wrap: el delta de heading se normaliza en [-π, π] antes de
 * calcular el promedio, evitando el salto catastrófico al cruzar 0°/360°
 * (p.ej. 359° → 1° daría un promedio de 180° sin esta corrección).
 *
 * IMPORTANTE: llama a set_pose({x, y, heading}) antes de la auton
 * para definir el origen correcto según la alianza.
 */
void vortex::Drivetrain::update_odom() {

    // 1. Guardar heading previo antes de actualizarlo
    double prev_heading_rad = current_pose.heading * M_PI / 180.0;

    // 2. Calcular desplazamiento lineal de cada rueda (cm)
    double pos_l  = (rot_sensor_l.get_position() / 36000.0) * circumference * factor;
    double pos_r  = (rot_sensor_r.get_position() / 36000.0) * circumference * factor;

    double delta_dist = ((pos_l - odom_prev_l) + (pos_r - odom_prev_r)) / 2.0;
    odom_prev_l = pos_l;
    odom_prev_r = pos_r;

    // 3. Actualizar heading desde la IMU (más preciso que integrar encoders)
    current_pose.heading = imu.get_heading();
    double current_heading_rad = current_pose.heading * M_PI / 180.0;

    // 4. Delta de heading normalizado en [-π, π] — fix del wrap 0°/360°
    double delta_heading = current_heading_rad - prev_heading_rad;
    if (delta_heading >  M_PI) delta_heading -= 2.0 * M_PI;
    if (delta_heading < -M_PI) delta_heading += 2.0 * M_PI;

    // 5. Ángulo promedio del arco recorrido en este ciclo
    double avg_heading_rad = prev_heading_rad + delta_heading / 2.0;

    // 6. Proyectar desplazamiento — Norte = +Y, Este = +X (convención de campo VEX)
    current_pose.x += delta_dist * std::sin(avg_heading_rad);
    current_pose.y += delta_dist * std::cos(avg_heading_rad);
}

// ─────────────────────────────────────────────
//  AUTONOMÍA — ALTO NIVEL
// ─────────────────────────────────────────────

// face_angle(): gira hasta apuntar a un heading ABSOLUTO del campo.
void vortex::Drivetrain::face_angle(double target_heading, int max_vel) {

    double error      = normalize_angle(target_heading - imu.get_heading());
    double last_error = error;
    double integral   = 0.0;

    uint32_t start_time = pros::millis();

    while (std::abs(error) > 1.0 && (pros::millis() - start_time < 3000)) {

        error = normalize_angle(target_heading - imu.get_heading());

        // 1. Mitigación de Integral Windup: Solo acumular si el error es menor a 15 grados
        if (std::abs(error) < 15.0) {
            integral += error;
        } else {
            integral = 0.0; // Limpiar si está muy lejos
        }

        // 2. Derivativa directa sin doble normalización
        double derivative = error - last_error;

        double output =
            (error      * config.turn_pid.kp) +
            (integral   * config.turn_pid.ki) +
            (derivative * config.turn_pid.kd);

        // 3. Compensación de fricción ANTES del clamp
        if (std::abs(error) > 1.0 && std::abs(output) < config.params.min_turn) {
            output = (output >= 0) ? config.params.min_turn : -config.params.min_turn;
        }

        // 4. El Clamp ahora protege correctamente los límites reales del motor
        int command = static_cast<int>(std::clamp(output, static_cast<double>(-max_vel), static_cast<double>(max_vel)));

        left.move(command);
        right.move(-command);

        last_error = error;
        pros::delay(20);
    }

    if (pros::competition::is_autonomous()) {

        hold();

    } else {

        hold(150);

        left.move(0);
        right.move(0);
    }
}

// ── move_centimeters ─────────────────────────
/*
 * Usa imu->get_heading() para la corrección de rumbo, consistente con
 * face_angle(). El error de ángulo también pasa por normalize_angle()
 * para manejar correctamente el wrap cerca de 0°/360°.
 */
void vortex::Drivetrain::move_centimeters(int target_cm, int max_vel) {

    double start_dist = get_distance();

    double target_angle = imu.get_heading();

    double current_speed = 0.0;

    double integral   = 0.0;
    double last_error = static_cast<double>(target_cm);

    constexpr double dt = 0.02;

    // ─────────────────────────────
    //  Tolerancias y límites
    // ─────────────────────────────

    constexpr double ANGLE_DEADBAND  = 0.2;

    constexpr double INTEGRAL_ZONE   = 12.0;
    constexpr double INTEGRAL_LIMIT  = 40.0;

    // Desaceleración más agresiva
    const double DECEL_STEP = config.params.accel_st * 2.5;

    uint32_t start_time = pros::millis();

    while ((pros::millis() - start_time) < 5000) {

        // ─────────────────────────────
        //  Error de distancia
        // ─────────────────────────────

        double current_dist = get_distance() - start_dist;

        double dist_error = target_cm - current_dist;

        // Fin del movimiento
        if (std::abs(dist_error) <= 1.0) {
            break;
        }

        // ─────────────────────────────
        //  Integral con anti-windup
        // ─────────────────────────────

        if (std::abs(dist_error) < INTEGRAL_ZONE) {

            integral += dist_error * dt;

            integral = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        } else {
            integral = 0.0;
        }

        // ─────────────────────────────
        //  Derivada
        // ─────────────────────────────

        double derivative = (dist_error - last_error) / dt;

        // ─────────────────────────────
        //  PID translacional
        // ─────────────────────────────

        double target_speed =
            (dist_error * config.move_pid.kp) +
            (integral   * config.move_pid.ki) +
            (derivative * config.move_pid.kd);

        target_speed = std::clamp(target_speed, static_cast<double>(-max_vel), static_cast<double>( max_vel));

        // ─────────────────────────────
        //  Compensación de fricción
        // ─────────────────────────────

        if (std::abs(target_speed) < config.params.min_move && std::abs(dist_error) > 1.0) {

            target_speed = (target_speed >= 0.0) ? config.params.min_move : -config.params.min_move;
        }

        // ─────────────────────────────
        //  Slew rate bidireccional
        // ─────────────────────────────

        double step = (std::abs(target_speed) < std::abs(current_speed)) ? DECEL_STEP : config.params.accel_st;

        double delta = target_speed - current_speed;

        delta = std::clamp(delta, -step, step);

        current_speed += delta;

        // ─────────────────────────────
        //  Corrección angular
        // ─────────────────────────────

        double angle_error = normalize_angle(target_angle - imu.get_heading());

        if (std::abs(angle_error) < ANGLE_DEADBAND) 
            angle_error *= 0.3;

        int steer_output = static_cast<int>(angle_error * config.params.angle_kp);

        // ─────────────────────────────
        //  Salida final
        // ─────────────────────────────

        int left_command = static_cast<int>(current_speed) + steer_output;

        int right_command = static_cast<int>(current_speed) - steer_output;

        printf(
        "heading: %.2f | angle_error: %.2f | steer: %d | speed: %.2f\n",
        imu.get_heading(),
        angle_error,
        steer_output,
        current_speed
        );

        left.move(std::clamp(left_command, -max_vel, max_vel));

        right.move(std::clamp(right_command, -max_vel, max_vel));

        last_error = dist_error;

        pros::delay(20);
    }

    // ─────────────────────────────
    //  Frenado final
    // ─────────────────────────────

    if (pros::competition::is_autonomous()) {

        hold();

    } else {

        hold(150);

        left.move(0);
        right.move(0);
    }
}

/*
 * drive_to_point(): mueve el robot a una coordenada (x, y) del campo.
 *
 * Algoritmo:
 *   1. Calcula el ángulo hacia el objetivo con atan2.
 *   2. Gira hasta apuntar (face_angle).
 *   3. Avanza la distancia euclidiana hasta el punto.
 *
 * Limitaciones conocidas:
 *   - Trayectoria en dos fases (giro → línea recta), no curva continua.
 *   - Para trayectorias más suaves, considerar Pure Pursuit en el futuro.
 *   - Depende de update_odom() corriendo en background para que
 *     current_pose esté actualizado.
 */
void vortex::Drivetrain::drive_to_point(double x, double y, int max_vel) {

    double dx = x - current_pose.x;
    double dy = y - current_pose.y;

    double target_heading = std::atan2(dx, dy) * 180.0 / M_PI;

    double distance = std::sqrt((dx * dx) + (dy * dy));

    double angle_error = normalize_angle(target_heading - imu.get_heading());

    // Reversa inteligente
    bool reverse = false;

    if (std::abs(angle_error) > 90.0) {

        reverse = true;
        distance = -distance;
        target_heading = normalize_angle(target_heading + 180.0);
    }

    face_angle(target_heading, max_vel);

    move_centimeters(static_cast<int>(distance), max_vel);
}
