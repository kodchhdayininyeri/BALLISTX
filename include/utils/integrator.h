#ifndef BALLISTX_INTEGRATOR_H
#define BALLISTX_INTEGRATOR_H

#include "utils/vec3.h"
#include <functional>

namespace ballistx {

/**
 * @brief State structure for numerical integration
 *
 * Contains position and velocity for 6-DOF simulation
 */
struct State {
    Vec3 position;      // meters
    Vec3 velocity;      // m/s

    State() = default;
    State(Vec3 pos, Vec3 vel) : position(pos), velocity(vel) {}
};

/**
 * @brief Runge-Kutta 4th Order Integrator
 *
 * Much more accurate than Euler integration.
 * Error: O(dt^4) vs Euler's O(dt)
 *
 * Usage:
 *   RK4Integrator rk4;
 *   State current = ...;
 *   auto derivative = [](const State& s, double t) -> Vec3 {
 *       return calculate_acceleration(s, t);
 *   };
 *   State next = rk4.step(current, t, dt, derivative);
 */
class RK4Integrator {
public:
    /**
     * @brief Perform one RK4 integration step
     *
     * @param s Current state (position, velocity)
     * @param t Current time
     * @param dt Time step
     * @param f Derivative function: f(state, time) -> acceleration
     * @return Next state
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
     * @brief Integrate over multiple steps
     *
     * @param initial Initial state
     * @param t0 Start time
     * @param t_end End time
     * @param dt Time step
     * @param f Derivative function
     * @param callback Optional callback after each step
     * @return Final state
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
