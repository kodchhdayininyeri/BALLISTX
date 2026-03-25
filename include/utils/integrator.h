#ifndef BALLISTX_INTEGRATOR_H
#define BALLISTX_INTEGRATOR_H

#include "utils/vec3.h"
#include <functional>

namespace ballistx {

/**
 * @brief State structure for numerical integration
 *
 * Simple state container for 6-DOF (degrees of freedom) simulation.
 * Contains position and velocity for second-order differential equations.
 *
 * @example
 * @code
 * State initial_state(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0));
 * // Position: (0, 0, 0), Velocity: (100, 0, 0) m/s
 * @endcode
 *
 * @see RK4Integrator for numerical integration
 * @see State6DOF for full 6-DOF state with orientation
 */
struct State {
    Vec3 position;  ///< Position vector [meters]
    Vec3 velocity;  ///< Velocity vector [m/seconds]

    /**
     * @brief Default constructor - initializes to zero
     */
    State() = default;

    /**
     * @brief Parameterized constructor
     *
     * @param pos Initial position
     * @param vel Initial velocity
     */
    State(Vec3 pos, Vec3 vel) : position(pos), velocity(vel) {}
};

/**
 * @brief Runge-Kutta 4th Order numerical integrator
 *
 * High-accuracy numerical integrator for solving differential equations.
 * Fourth-order Runge-Kutta (RK4) provides O(dt⁴) accuracy compared to
 * Euler's O(dt) method, making it suitable for ballistics simulations.
 *
 * **Advantages:**
 * - Fourth-order accuracy (error ∝ dt⁴)
 * - Self-starting (no history needed)
 * - Stable for a wide range of time steps
 * - Computationally efficient (4 function evaluations per step)
 *
 * **RK4 Algorithm:**
 * ```
 * k1 = f(t, y)
 * k2 = f(t + dt/2, y + dt*k1/2)
 * k3 = f(t + dt/2, y + dt*k2/2)
 * k4 = f(t + dt, y + dt*k3)
 * y_{n+1} = y_n + (dt/6)(k1 + 2k2 + 2k3 + k4)
 * ```
 *
 * @example
 * @code
 * RK4Integrator rk4;
 *
 * // Initial state: projectile at origin, 250 m/s horizontal
 * State current(Vec3(0.0, 0.0, 0.0), Vec3(250.0, 0.0, 0.0));
 *
 * // Define derivative function (acceleration)
 * auto acceleration = [](const State& s, double t) -> Vec3 {
 *     return Vec3(0.0, -9.81, 0.0);  // Gravity only
 * };
 *
 * // Integrate for 1 second with 0.01s time steps
 * State next = rk4.step(current, 0.0, 0.01, acceleration);
 *
 * // Or integrate over a duration with callback
 * State final = rk4.integrate(current, 0.0, 10.0, 0.01,
 *     acceleration,
 *     [](const State& s, double t) {
 *         std::cout << "t=" << t << " pos=" << s.position << std::endl;
 *     }
 * );
 * @endcode
 *
 * @see State for state structure
 * @see State6DOF for full 6-DOF integration
 */
class RK4Integrator {
public:
    /**
     * @brief Perform one RK4 integration step
     *
     * Advances the state forward by time `dt` using fourth-order
     * Runge-Kutta integration. The derivative function should return
     * the acceleration (second derivative of position).
     *
     * @param s Current state (position and velocity)
     * @param t Current simulation time
     * @param dt Time step to advance [seconds]
     * @param f Derivative function: f(state, time) → acceleration
     * @return Next state after time step
     *
     * @example
     * @code
     * RK4Integrator rk4;
     * State current(Vec3(0.0, 100.0, 0.0), Vec3(50.0, 0.0, 0.0));
     *
     * auto gravity = [](const State& s, double t) -> Vec3 {
     *     return Vec3(0.0, -9.81, 0.0);
     * };
     *
     * State next = rk4.step(current, 0.0, 0.1, gravity);
     * @endcode
     */
    State step(const State& s, double t, double dt,
               std::function<Vec3(const State&, double)> f) const {

        // k1 - slope at beginning
        Vec3 k1v = f(s, t) * dt;
        Vec3 k1x = s.velocity * dt;

        // k2 - slope at midpoint using k1
        State s2 = {s.position + k1x * 0.5, s.velocity + k1v * 0.5};
        Vec3 k2v = f(s2, t + dt * 0.5) * dt;
        Vec3 k2x = s2.velocity * dt;

        // k3 - slope at midpoint using k2
        State s3 = {s.position + k2x * 0.5, s.velocity + k2v * 0.5};
        Vec3 k3v = f(s3, t + dt * 0.5) * dt;
        Vec3 k3x = s3.velocity * dt;

        // k4 - slope at end using k3
        State s4 = {s.position + k3x, s.velocity + k3v};
        Vec3 k4v = f(s4, t + dt) * dt;
        Vec3 k4x = s4.velocity * dt;

        // Weighted average of slopes
        return {
            s.position + (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (1.0 / 6.0),
            s.velocity + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (1.0 / 6.0)
        };
    }

    /**
     * @brief Integrate over multiple time steps
     *
     * Performs repeated RK4 steps from t0 to t_end with fixed time step dt.
     * Optionally calls a callback function after each step for logging
     * or data collection.
     *
     * @param initial Initial state at time t0
     * @param t0 Start time [seconds]
     * @param t_end End time [seconds]
     * @param dt Time step for integration [seconds]
     * @param f Derivative function: f(state, time) → acceleration
     * @param callback Optional function called after each step with (state, time)
     * @return Final state at time t_end
     *
     * @example
     * @code
     * RK4Integrator rk4;
     * State start(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 100.0, 0.0));
     *
     * auto accel = [](const State& s, double t) -> Vec3 {
     *     return Vec3(0.0, -9.81, 0.0);
     * };
     *
     * // Integrate for 5 seconds, logging every step
     * State result = rk4.integrate(start, 0.0, 5.0, 0.01,
     *     accel,
     *     [](const State& s, double t) {
     *         std::cout << "t=" << t << ", y=" << s.position.y << std::endl;
     *     }
     * );
     * @endcode
     */
    State integrate(const State& initial, double t0, double t_end, double dt,
                   std::function<Vec3(const State&, double)> f,
                   std::function<void(const State&, double)> callback = nullptr) const {
        State current = initial;
        double t = t0;

        while (t < t_end) {
            current = step(current, t, dt, f);
            t += dt;

            if (callback) {
                callback(current, t);
            }
        }

        return current;
    }
};

} // namespace ballistx

#endif // BALLISTX_INTEGRATOR_H
