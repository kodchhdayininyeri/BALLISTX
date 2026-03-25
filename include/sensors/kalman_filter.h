#ifndef BALLISTX_KALMAN_FILTER_H
#define BALLISTX_KALMAN_FILTER_H

#include "ballistics/state_6dof.h"
#include "utils/vec3.h"
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief Radar measurement vector for target tracking
 *
 * Represents noisy radar measurements in spherical coordinates.
 * Real radar measurements always contain noise from various sources:
 * - Thermal noise in receiver electronics
 * - Atmospheric effects and multipath propagation
 * - Target radar cross-section fluctuations
 * - Quantization errors in signal processing
 *
 * **Measurement Components:**
 * - Range (r): Distance from radar to target [m]
 * - Azimuth (θ): Horizontal angle from North [rad]
 * - Elevation (φ): Vertical angle from horizon [rad]
 * - Range rate (ṙ): Radial velocity (Doppler) [m/s]
 *
 * @example
 * @code
 * // Simulate a noisy radar measurement
 * Vec3 true_pos(5000.0, 2000.0, 3000.0);
 * Vec3 true_vel(250.0, 50.0, 0.0);
 *
 * RadarMeasurement z;
 * z.range = true_pos.magnitude() + 50.0;           // 50m noise
 * z.azimuth = std::atan2(true_pos.x, true_pos.z) + 0.001;  // ~0.06° noise
 * z.elevation = std::asin(true_pos.y / true_pos.magnitude()) + 0.0005;
 * z.range_rate = true_vel.dot(true_pos.normalized()) + 2.0;  // 2 m/s noise
 *
 * // Create measurement covariance (measurement accuracy)
 * RadarMeasurement R;
 * R.range = 100.0;      // 100m std dev (~10m RMS)
 * R.azimuth = 0.01;     // ~0.57° std dev
 * R.elevation = 0.005;  // ~0.29° std dev
 * R.range_rate = 5.0;   // 5 m/s std dev
 * @endcode
 */
struct RadarMeasurement {
    double range = 0.0;        ///< Distance to target [m]
    double azimuth = 0.0;      ///< Horizontal angle from North [rad]
    double elevation = 0.0;    ///< Vertical angle from horizon [rad]
    double range_rate = 0.0;   ///< Radial velocity [m/s]

    /**
     * @brief Default constructor - zero measurement
     */
    RadarMeasurement() = default;

    /**
     * @brief Create measurement from target state
     *
     * Computes the ideal (noise-free) radar measurement from a target state.
     *
     * @param state Target state in world coordinates
     *
     * @example
     * @code
     * State6DOF target(pos, vel, quat, ang_vel);
     * RadarMeasurement z = RadarMeasurement::from_state(target);
     * // Add noise...
     * @endcode
     */
    static RadarMeasurement from_state(const State6DOF& state) {
        RadarMeasurement z;
        Vec3 pos = state.get_position();
        Vec3 vel = state.get_velocity();

        double r = pos.magnitude();
        z.range = std::max(0.0, r);  // Range cannot be negative
        z.azimuth = std::atan2(pos.x, pos.z);
        z.elevation = std::asin(pos.y / (r + 1e-10));
        z.range_rate = vel.dot(pos.normalized());

        return z;
    }

    /**
     * @brief Convert measurement to Cartesian position
     *
     * Converts spherical coordinates to Cartesian (x, y, z).
     * Note: This gives only position, not full state.
     *
     * @return Position vector [m]
     */
    Vec3 to_cartesian() const {
        double r_cos_el = range * std::cos(elevation);
        return Vec3(
            r_cos_el * std::sin(azimuth),
            range * std::sin(elevation),
            r_cos_el * std::cos(azimuth)
        );
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const RadarMeasurement& z) {
        os << "Radar: r=" << z.range << "m, az=" << (z.azimuth * 180.0 / M_PI)
           << "°, el=" << (z.elevation * 180.0 / M_PI) << "°, ṙ=" << z.range_rate << "m/s";
        return os;
    }
};

/**
 * @brief Extended Kalman Filter for 6-DOF target tracking
 *
 * The Extended Kalman Filter (EKF) is the optimal recursive estimator
 * for nonlinear systems. It extends the standard Kalman Filter to handle
 * nonlinear dynamics and measurement models by linearizing around the
 * current state estimate.
 *
 * **EKF Algorithm:**
 *
 * 1. **Prediction Step (Time Update):**
 *    - Predict state: x̂ₖ₋₁ = f(x̂ₖ₋₁, uₖ₋₁)
 *    - Predict covariance: P̂ₖ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ
 *
 * 2. **Update Step (Measurement Update):**
 *    - Compute innovation: ỹₖ = zₖ - h(x̂ₖ)
 *    - Compute innovation covariance: Sₖ = Hₖ P̂ₖ Hₖᵀ + Rₖ
 *    - Compute Kalman gain: Kₖ = P̂ₖ Hₖᵀ Sₖ⁻¹
 *    - Update state: x̂ₖ = x̂ₖ + Kₖ ỹₖ
 *    - Update covariance: Pₖ = (I - Kₖ Hₖ) P̂ₖ
 *
 * Where:
 * - F = State transition Jacobian: ∂f/∂x
 * - H = Measurement Jacobian: ∂h/∂x
 * - Q = Process noise covariance
 * - R = Measurement noise covariance
 *
 * @example
 * @code
 * // Initialize EKF for missile tracking
 * ExtendedKalmanFilter ekf;
 *
 * // Set initial state estimate
 * State6DOF initial_estimate(
 *     Vec3(0.0, 1000.0, 0.0),      // Initial position guess
 *     Vec3(200.0, 0.0, 0.0),       // Initial velocity guess
 *     Quaternion::identity(),
 *     Vec3(0.0, 0.0, 0.0)
 * );
 * ekf.set_state(initial_estimate);
 *
 * // Set initial covariance (uncertainty in estimate)
 * ekf.set_position_covariance(100.0);     // 100m position uncertainty
 * ekf.set_velocity_covariance(10.0);      // 10 m/s velocity uncertainty
 *
 * // Set process noise (model uncertainty)
 * ekf.set_process_noise(0.1, 0.01);       // Position/velocity process noise
 *
 * // Set measurement noise (radar accuracy)
 * RadarMeasurement R;
 * R.range = 50.0;        // 50m range std
 * R.azimuth = 0.005;     // ~0.3° azimuth std
 * R.elevation = 0.003;   // ~0.2° elevation std
 * R.range_rate = 2.0;    // 2 m/s range rate std
 * ekf.set_measurement_noise(R);
 *
 * // Main tracking loop
 * while (tracking) {
 *     // 1. Predict step (propagate state forward)
 *     double dt = 0.01;  // 10ms timestep
 *     ekf.predict(dt, process_model);
 *
 *     // 2. Get noisy radar measurement
 *     RadarMeasurement z = radar.get_measurement();
 *
 *     // 3. Update step (incorporate measurement)
 *     ekf.update(z);
 *
 *     // 4. Get filtered estimate
 *     State6DOF estimate = ekf.get_state();
 *     Vec3 pos = estimate.get_position();
 *     Vec3 vel = estimate.get_velocity();
 *
 *     // Get estimate uncertainty
 *     double pos_uncertainty = ekf.get_position_uncertainty();
 * }
 * @endcode
 *
 * @see State6DOF for state vector representation
 * @see RadarMeasurement for measurement model
 */
class ExtendedKalmanFilter {
public:
    // State and covariance sizes
    static constexpr size_t STATE_SIZE = 6;  // px, py, pz, vx, vy, vz (simplified)
    static constexpr size_t MEAS_SIZE = 4;   // range, az, el, range_rate

    /**
     * @brief Process model function type
     *
     * Function that computes state derivatives: dx/dt = f(x, t)
     * Used in the prediction step to propagate the state forward.
     *
     * @param position Current position
     * @param velocity Current velocity
     * @param time Current simulation time
     * @return Acceleration vector
     */
    using ProcessModel = std::function<Vec3(Vec3, Vec3, double)>;

    // === CONSTRUCTORS ===

    /**
     * @brief Default constructor
     *
     * Initializes EKF with zero state and identity covariance.
     * Process and measurement noise must be set before use.
     */
    ExtendedKalmanFilter();

    /**
     * @brief Constructor with initial state
     *
     * @param initial_state Initial state estimate
     * @param initial_cov Initial covariance matrix (diagonal values)
     */
    ExtendedKalmanFilter(const State6DOF& initial_state, double initial_cov = 1.0);

    // === STATE ACCESSORS ===

    /**
     * @brief Get current state estimate
     *
     * @return Filtered state estimate
     */
    State6DOF get_state() const {
        return State6DOF(position_, velocity_, orientation_, angular_velocity_);
    }

    /**
     * @brief Set state estimate
     *
     * @param state New state estimate
     */
    void set_state(const State6DOF& state) {
        position_ = state.get_position();
        velocity_ = state.get_velocity();
        orientation_ = state.get_orientation();
        angular_velocity_ = state.get_angular_velocity();
    }

    /**
     * @brief Get position estimate
     */
    Vec3 get_position() const { return position_; }

    /**
     * @brief Get velocity estimate
     */
    Vec3 get_velocity() const { return velocity_; }

    /**
     * @brief Get orientation estimate
     */
    Quaternion get_orientation() const { return orientation_; }

    /**
     * @brief Get angular velocity estimate
     */
    Vec3 get_angular_velocity() const { return angular_velocity_; }

    // === COVARIANCE ACCESSORS ===

    /**
     * @brief Get full covariance matrix
     *
     * @return Flattened covariance matrix (STATE_SIZE × STATE_SIZE)
     */
    const std::array<double, STATE_SIZE * STATE_SIZE>& get_covariance() const {
        return P_;
    }

    /**
     * @brief Set covariance matrix
     *
     * @param P New covariance matrix (flattened)
     */
    void set_covariance(const std::array<double, STATE_SIZE * STATE_SIZE>& P) {
        P_ = P;
    }

    /**
     * @brief Get position uncertainty (1-sigma)
     *
     * Returns the square root of the largest position variance.
     *
     * @return Position uncertainty [m]
     */
    double get_position_uncertainty() const;

    /**
     * @brief Get velocity uncertainty (1-sigma)
     *
     * @return Velocity uncertainty [m/s]
     */
    double get_velocity_uncertainty() const;

    /**
     * @brief Set diagonal covariance values
     *
     * Convenience method for setting initial uncertainty.
     *
     * @param pos_cov Position variance [m²]
     * @param vel_cov Velocity variance [m²/s²]
     */
    void set_diagonal_covariance(double pos_cov = 1.0, double vel_cov = 1.0);

    /**
     * @brief Set position covariance (variance)
     *
     * @param std_dev Standard deviation [m]
     */
    void set_position_covariance(double std_dev);

    /**
     * @brief Set velocity covariance (variance)
     *
     * @param std_dev Standard deviation [m/s]
     */
    void set_velocity_covariance(double std_dev);

    // === NOISE CONFIGURATION ===

    /**
     * @brief Set process noise covariance
     *
     * Process noise represents uncertainty in the system model.
     * Higher values make the filter trust measurements more.
     *
     * @param pos_noise Position process noise variance [m²/s]
     * @param vel_noise Velocity process noise variance [m²/s³]
     */
    void set_process_noise(double pos_noise = 0.1, double vel_noise = 0.01);

    /**
     * @brief Set measurement noise covariance
     *
     * Measurement noise represents sensor accuracy.
     * Should be set based on radar specifications.
     *
     * @param R Measurement noise values (variances)
     */
    void set_measurement_noise(const RadarMeasurement& R);

    /**
     * @brief Get measurement noise covariance
     *
     * @return Measurement noise (variances)
     */
    RadarMeasurement get_measurement_noise() const {
        return measurement_noise_;
    }

    // === PREDICTION STEP ===

    /**
     * @brief Predict step (time update)
     *
     * Propagates state and covariance forward by time dt.
     * Uses the process model and computes the state transition Jacobian.
     *
     * **Prediction equations:**
     * - x̂ₖ = f(x̂ₖ₋₁, uₖ₋₁, dt)
     * - P̂ₖ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ
     *
     * @param dt Time step [s]
     * @param process_model Function that computes acceleration
     *
     * @example
     * @code
     * // Define process model (acceleration from forces)
     * auto process_model = [](Vec3 pos, Vec3 vel, double t) -> Vec3 {
     *     // Calculate acceleration from gravity, drag, etc.
     *     Vec3 gravity(0.0, -9.81, 0.0);
     *     double speed = vel.magnitude();
     *     Vec3 drag = vel * (-0.001 * speed);
     *     return gravity + drag;
     * };
     *
     * ekf.predict(0.01, process_model);
     * @endcode
     */
    void predict(double dt, const ProcessModel& process_model);

    /**
     * @brief Predict with simple kinematic model
     *
     * Uses a simple constant-velocity model for prediction.
     * Suitable when detailed dynamics are unknown.
     *
     * @param dt Time step [s]
     */
    void predict_kinematic(double dt);

    // === UPDATE STEP ===

    /**
     * @brief Update step (measurement update)
     *
     * Incorporates a radar measurement to correct the state estimate.
     * Computes the measurement Jacobian and Kalman gain.
     *
     * **Update equations:**
     * - Innovation: ỹ = z - h(x̂)
     * - Innovation covariance: S = H P̂ Hᵀ + R
     * - Kalman gain: K = P̂ Hᵀ S⁻¹
     * - Updated state: x̂ = x̂ + K ỹ
     * - Updated covariance: P = (I - K H) P̂
     *
     * @param z Radar measurement
     *
     * @example
     * @code
     * // Get measurement from radar
     * RadarMeasurement z;
     * z.range = 5234.5;
     * z.azimuth = 0.523;
     * z.elevation = 0.312;
     * z.range_rate = -45.2;
     *
     * // Update filter
     * ekf.update(z);
     * @endcode
     */
    void update(const RadarMeasurement& z);

    // === UTILITY ===

    /**
     * @brief Reset filter to initial state
     *
     * @param initial_state New initial state
     */
    void reset(const State6DOF& initial_state);

    /**
     * @brief Get innovation (measurement residual)
     *
     * Returns the difference between the last measurement and prediction.
     * Useful for monitoring filter performance.
     *
     * @return Last innovation vector
     */
    const std::array<double, MEAS_SIZE>& get_innovation() const { return innovation_; }

    /**
     * @brief Check if filter is healthy
     *
     * Checks for numerical issues (NaN, infinite values, etc.)
     *
     * @return true if filter is healthy
     */
    bool is_healthy() const;

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const ExtendedKalmanFilter& ekf) {
        os << "EKF State:\n";
        os << "  Position: " << ekf.get_position() << "\n";
        os << "  Velocity: " << ekf.get_velocity() << "\n";
        os << "  Pos Uncertainty: " << ekf.get_position_uncertainty() << " m\n";
        os << "  Vel Uncertainty: " << ekf.get_velocity_uncertainty() << " m/s\n";
        return os;
    }

private:
    Vec3 position_;          ///< Position estimate x̂
    Vec3 velocity_;          ///< Velocity estimate v̂
    Quaternion orientation_; ///< Orientation estimate q̂
    Vec3 angular_velocity_;  ///< Angular velocity estimate ω̂

    std::array<double, STATE_SIZE * STATE_SIZE> P_;        ///< Covariance matrix P
    std::array<double, STATE_SIZE * STATE_SIZE> Q_;        ///< Process noise Q
    std::array<double, MEAS_SIZE * MEAS_SIZE> R_;          ///< Measurement noise R
    RadarMeasurement measurement_noise_;                    ///< Measurement noise (struct)

    std::array<double, MEAS_SIZE> innovation_;             ///< Last innovation ỹ

    // === Helper functions ===

    /**
     * @brief Matrix operations for small matrices
     */
    static void mat_mult_6x6(const double A[36], const double B[36], double C[36]);
    static void mat_transpose_6x6(const double A[36], double AT[36]);
    static void mat_mult_4x6_6x6(const double A[24], const double B[36], double C[24]);
    static void mat_mult_6x6_6x4(const double A[36], const double B[24], double C[24]);
    static void mat_mult_4x6_6x4(const double A[24], const double B[24], double C[16]);

    /**
     * @brief Matrix inversion for 4x4 symmetric positive-definite matrix
     */
    static bool mat_inv_4x4_spd(const double A[16], double A_inv[16]);

    /**
     * @brief Compute measurement Jacobian H
     */
    void compute_measurement_jacobian(double H[24]) const;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline ExtendedKalmanFilter::ExtendedKalmanFilter()
    : position_(), velocity_(), orientation_(), angular_velocity_(),
      P_{}, Q_{}, R_{}, measurement_noise_{}, innovation_{} {
    set_diagonal_covariance(100.0, 10.0);
    set_process_noise(0.1, 0.01);
}

inline ExtendedKalmanFilter::ExtendedKalmanFilter(const State6DOF& initial_state, double initial_cov)
    : position_(initial_state.get_position()),
      velocity_(initial_state.get_velocity()),
      orientation_(initial_state.get_orientation()),
      angular_velocity_(initial_state.get_angular_velocity()),
      P_{}, Q_{}, R_{}, measurement_noise_{}, innovation_{} {
    set_diagonal_covariance(initial_cov, initial_cov);
    set_process_noise(0.1, 0.01);
}

inline double ExtendedKalmanFilter::get_position_uncertainty() const {
    double max_var = P_[0 * 6 + 0];  // px
    max_var = std::max(max_var, P_[1 * 6 + 1]);  // py
    max_var = std::max(max_var, P_[2 * 6 + 2]);  // pz
    return std::sqrt(max_var);
}

inline double ExtendedKalmanFilter::get_velocity_uncertainty() const {
    double max_var = P_[3 * 6 + 3];  // vx
    max_var = std::max(max_var, P_[4 * 6 + 4]);  // vy
    max_var = std::max(max_var, P_[5 * 6 + 5]);  // vz
    return std::sqrt(max_var);
}

inline void ExtendedKalmanFilter::set_diagonal_covariance(double pos_cov, double vel_cov) {
    P_.fill(0.0);
    P_[0 * 6 + 0] = pos_cov;  // px
    P_[1 * 6 + 1] = pos_cov;  // py
    P_[2 * 6 + 2] = pos_cov;  // pz
    P_[3 * 6 + 3] = vel_cov;  // vx
    P_[4 * 6 + 4] = vel_cov;  // vy
    P_[5 * 6 + 5] = vel_cov;  // vz
}

inline void ExtendedKalmanFilter::set_position_covariance(double std_dev) {
    double var = std_dev * std_dev;
    P_[0 * 6 + 0] = var;
    P_[1 * 6 + 1] = var;
    P_[2 * 6 + 2] = var;
}

inline void ExtendedKalmanFilter::set_velocity_covariance(double std_dev) {
    double var = std_dev * std_dev;
    P_[3 * 6 + 3] = var;
    P_[4 * 6 + 4] = var;
    P_[5 * 6 + 5] = var;
}

inline void ExtendedKalmanFilter::set_process_noise(double pos_noise, double vel_noise) {
    Q_.fill(0.0);
    Q_[0 * 6 + 0] = pos_noise;
    Q_[1 * 6 + 1] = pos_noise;
    Q_[2 * 6 + 2] = pos_noise;
    Q_[3 * 6 + 3] = vel_noise;
    Q_[4 * 6 + 4] = vel_noise;
    Q_[5 * 6 + 5] = vel_noise;
}

inline void ExtendedKalmanFilter::set_measurement_noise(const RadarMeasurement& R) {
    measurement_noise_ = R;
    R_.fill(0.0);
    R_[0] = R.range * R.range;
    R_[5] = R.azimuth * R.azimuth;
    R_[10] = R.elevation * R.elevation;
    R_[15] = R.range_rate * R.range_rate;
}

inline void ExtendedKalmanFilter::predict(double dt, const ProcessModel& process_model) {
    // 1. Predict state: x̂ₖ = x̂ₖ₋₁ + v̂ₖ₋₁ * dt
    position_ = position_ + velocity_ * dt;

    // 2. Get acceleration from process model
    Vec3 accel = process_model(position_, velocity_, 0.0);
    velocity_ = velocity_ + accel * dt;

    // 3. Predict covariance: P̂ = F P Fᵀ + Q
    // State transition matrix F (for constant velocity with acceleration)
    // x' = x + vx*dt + 0.5*ax*dt²
    // v' = vx + ax*dt

    double F[36] = {0};
    F[0 * 6 + 0] = 1.0;  // px
    F[1 * 6 + 1] = 1.0;  // py
    F[2 * 6 + 2] = 1.0;  // pz
    F[3 * 6 + 3] = 1.0;  // vx
    F[4 * 6 + 4] = 1.0;  // vy
    F[5 * 6 + 5] = 1.0;  // vz

    F[0 * 6 + 3] = dt;   // ∂px/∂vx
    F[1 * 6 + 4] = dt;   // ∂py/∂vy
    F[2 * 6 + 5] = dt;   // ∂pz/∂vz

    double FT[36];
    mat_transpose_6x6(F, FT);

    double FP[36];
    mat_mult_6x6(F, P_.data(), FP);

    double FPFT[36];
    mat_mult_6x6(FP, FT, FPFT);

    // P = F P Fᵀ + Q
    for (size_t i = 0; i < 36; ++i) {
        P_[i] = FPFT[i] + Q_[i];
    }
}

inline void ExtendedKalmanFilter::predict_kinematic(double dt) {
    // Simple constant velocity model
    position_ = position_ + velocity_ * dt;

    // F = [I  I*dt]
    //     [0   I  ]
    double F[36] = {0};
    for (size_t i = 0; i < 6; ++i) {
        F[i * 6 + i] = 1.0;
    }
    F[0 * 6 + 3] = dt;
    F[1 * 6 + 4] = dt;
    F[2 * 6 + 5] = dt;

    double FT[36];
    mat_transpose_6x6(F, FT);

    double FP[36];
    mat_mult_6x6(F, P_.data(), FP);

    double FPFT[36];
    mat_mult_6x6(FP, FT, FPFT);

    for (size_t i = 0; i < 36; ++i) {
        P_[i] = FPFT[i] + Q_[i];
    }
}

inline void ExtendedKalmanFilter::update(const RadarMeasurement& z) {
    // 1. Compute predicted measurement
    Vec3 pos_norm = position_.normalized();
    double r = position_.magnitude();

    double z_pred_range = r;
    double z_pred_azimuth = std::atan2(position_.x, position_.z);
    double z_pred_elevation = std::asin(position_.y / (r + 1e-10));
    double z_pred_range_rate = velocity_.dot(pos_norm);

    // 2. Compute innovation: ỹ = z - h(x̂)
    innovation_[0] = z.range - z_pred_range;
    innovation_[1] = z.azimuth - z_pred_azimuth;
    innovation_[2] = z.elevation - z_pred_elevation;
    innovation_[3] = z.range_rate - z_pred_range_rate;

    // Wrap azimuth to [-π, π]
    while (innovation_[1] > M_PI) innovation_[1] -= 2.0 * M_PI;
    while (innovation_[1] < -M_PI) innovation_[1] += 2.0 * M_PI;

    // 3. Compute measurement Jacobian H
    double H[24];  // 4x6
    compute_measurement_jacobian(H);

    // 4. Compute innovation covariance: S = H P Hᵀ + R
    double HP[24];
    mat_mult_4x6_6x6(H, P_.data(), HP);

    double HPHt[16];
    mat_mult_4x6_6x4(HP, H, HPHt);

    double S[16];
    for (size_t i = 0; i < 16; ++i) {
        S[i] = HPHt[i] + R_[i];
    }

    // 5. Compute Kalman gain: K = P Hᵀ S⁻¹
    double S_inv[16];
    if (!mat_inv_4x4_spd(S, S_inv)) {
        return;  // Skip update if matrix inversion fails
    }

    double PHt[24];  // 6x4
    mat_mult_6x6_6x4(P_.data(), H, PHt);  // P * Hᵀ

    double K[24];  // 6x4
    mat_mult_6x6_6x4(PHt, S_inv, K);  // P Hᵀ S⁻¹

    // 6. Update state: x̂ = x̂ + K ỹ
    double dx[6];
    for (size_t i = 0; i < 6; ++i) {
        dx[i] = 0.0;
        for (size_t j = 0; j < 4; ++j) {
            dx[i] += K[i * 4 + j] * innovation_[j];
        }
    }

    position_.x += dx[0];
    position_.y += dx[1];
    position_.z += dx[2];
    velocity_.x += dx[3];
    velocity_.y += dx[4];
    velocity_.z += dx[5];

    // 7. Update covariance: P = (I - K H) P
    double KH[36];
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                sum += K[i * 4 + k] * H[k * 6 + j];
            }
            KH[i * 6 + j] = sum;
        }
    }

    double I_KH[36];
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            double val = (i == j) ? 1.0 : 0.0;
            I_KH[i * 6 + j] = val - KH[i * 6 + j];
        }
    }

    double new_P[36];
    mat_mult_6x6(I_KH, P_.data(), new_P);
    for (size_t i = 0; i < 36; ++i) {
        P_[i] = new_P[i];
    }
}

inline void ExtendedKalmanFilter::reset(const State6DOF& initial_state) {
    position_ = initial_state.get_position();
    velocity_ = initial_state.get_velocity();
    orientation_ = initial_state.get_orientation();
    angular_velocity_ = initial_state.get_angular_velocity();
    set_diagonal_covariance(100.0, 10.0);
}

inline bool ExtendedKalmanFilter::is_healthy() const {
    // Check position is valid
    if (!std::isfinite(position_.x) || !std::isfinite(position_.y) || !std::isfinite(position_.z)) {
        return false;
    }

    // Check velocity is valid
    if (!std::isfinite(velocity_.x) || !std::isfinite(velocity_.y) || !std::isfinite(velocity_.z)) {
        return false;
    }

    // Check covariance is valid
    for (size_t i = 0; i < STATE_SIZE; ++i) {
        if (!std::isfinite(P_[i * STATE_SIZE + i]) || P_[i * STATE_SIZE + i] <= 0) {
            return false;
        }
    }

    return true;
}

// ============================================================================
// MATRIX HELPER FUNCTIONS
// ============================================================================

inline void ExtendedKalmanFilter::mat_mult_6x6(const double A[36], const double B[36], double C[36]) {
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < 6; ++k) {
                sum += A[i * 6 + k] * B[k * 6 + j];
            }
            C[i * 6 + j] = sum;
        }
    }
}

inline void ExtendedKalmanFilter::mat_transpose_6x6(const double A[36], double AT[36]) {
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            AT[i * 6 + j] = A[j * 6 + i];
        }
    }
}

inline void ExtendedKalmanFilter::mat_mult_4x6_6x6(const double A[24], const double B[36], double C[24]) {
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < 6; ++k) {
                sum += A[i * 6 + k] * B[k * 6 + j];
            }
            C[i * 6 + j] = sum;
        }
    }
}

inline void ExtendedKalmanFilter::mat_mult_6x6_6x4(const double A[36], const double B[24], double C[24]) {
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < 6; ++k) {
                sum += A[i * 6 + k] * B[k * 4 + j];
            }
            C[i * 4 + j] = sum;
        }
    }
}

inline void ExtendedKalmanFilter::mat_mult_4x6_6x4(const double A[24], const double B[24], double C[16]) {
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < 6; ++k) {
                sum += A[i * 6 + k] * B[k * 4 + j];
            }
            C[i * 4 + j] = sum;
        }
    }
}

inline bool ExtendedKalmanFilter::mat_inv_4x4_spd(const double A[16], double A_inv[16]) {
    // Cholesky decomposition: A = L Lᵀ
    double L[16] = {0};

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < j; ++k) {
                sum += L[i * 4 + k] * L[j * 4 + k];
            }

            if (i == j) {
                double diag = A[i * 4 + i] - sum;
                if (diag <= 1e-10) {
                    return false;
                }
                L[i * 4 + j] = std::sqrt(diag);
            } else {
                L[i * 4 + j] = (A[i * 4 + j] - sum) / L[j * 4 + j];
            }
        }
    }

    // Invert L
    double L_inv[16] = {0};

    // Forward substitution for identity matrix
    for (size_t col = 0; col < 4; ++col) {
        for (size_t i = col; i < 4; ++i) {
            if (i == col) {
                L_inv[i * 4 + col] = 1.0 / L[i * 4 + i];
            } else {
                double sum = 0.0;
                for (size_t k = col; k < i; ++k) {
                    sum += L[i * 4 + k] * L_inv[k * 4 + col];
                }
                L_inv[i * 4 + col] = -sum / L[i * 4 + i];
            }
        }
    }

    // A_inv = L_invᵀ L_inv
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                sum += L_inv[k * 4 + i] * L_inv[k * 4 + j];
            }
            A_inv[i * 4 + j] = sum;
        }
    }

    return true;
}

inline void ExtendedKalmanFilter::compute_measurement_jacobian(double H[24]) const {
    H[0] = 0.0;  // Initialize all to 0
    for (size_t i = 0; i < 24; ++i) H[i] = 0.0;

    double r = position_.magnitude() + 1e-10;
    double r2 = r * r;
    double r3 = r2 * r;

    Vec3 pos_norm = position_ / r;

    // Measurement 1: range = |p|
    // ∂range/∂p = p/|p|
    H[0 * 6 + 0] = pos_norm.x;
    H[0 * 6 + 1] = pos_norm.y;
    H[0 * 6 + 2] = pos_norm.z;

    // Measurement 2: azimuth = atan2(px, pz)
    double denom_az = position_.x * position_.x + position_.z * position_.z + 1e-10;
    H[1 * 6 + 0] = position_.z / denom_az;
    H[1 * 6 + 2] = -position_.x / denom_az;

    // Measurement 3: elevation = asin(py/r)
    double rho = std::sqrt(r2 - position_.y * position_.y + 1e-10);
    H[2 * 6 + 0] = -position_.x * position_.y / (r2 * rho);
    H[2 * 6 + 1] = rho / r2;
    H[2 * 6 + 2] = -position_.z * position_.y / (r2 * rho);

    // Measurement 4: range_rate = v · (p/r)
    double v_dot_p = velocity_.dot(position_);
    Vec3 dr_dp = (velocity_ / r) - (position_ * v_dot_p / r3);

    H[3 * 6 + 0] = dr_dp.x;
    H[3 * 6 + 1] = dr_dp.y;
    H[3 * 6 + 2] = dr_dp.z;
    H[3 * 6 + 3] = pos_norm.x;
    H[3 * 6 + 4] = pos_norm.y;
    H[3 * 6 + 5] = pos_norm.z;
}

} // namespace ballistx

#endif // BALLISTX_KALMAN_FILTER_H
