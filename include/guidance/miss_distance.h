#ifndef BALLISTX_MISS_DISTANCE_H
#define BALLISTX_MISS_DISTANCE_H

#include "utils/vec3.h"
#include "ballistics/state_6dof.h"
#include "guidance/guidance.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <tuple>
#include <iostream>
#include <limits>

namespace ballistx {

/**
 * @brief Closest Point of Approach (CPA) calculation
 *
 * Miss distance analysis for guidance quality assessment.
 *
 * CPA Metrics:
 * - Miss distance: Minimum distance between missile and target
 * - Time to CPA: When minimum distance occurs
 * - CPA position: Location where closest approach happens
 *
 * Uses linear prediction for fast calculation.
 */
class MissDistance {
public:
    /**
     * @brief CPA result structure
     */
    struct Result {
        double miss_distance;      // Minimum distance [m]
        double time_to_cpa;        // Time until CPA [s]
        Vec3 missile_position;    // Missile position at CPA [m]
        Vec3 target_position;     // Target position at CPA [m]
        double closing_velocity;  // Relative velocity at CPA [m/s]
        bool intercepted;         // Whether interception occurred

        Result() : miss_distance(0.0), time_to_cpa(0.0),
                 closing_velocity(0.0), intercepted(false) {}

        void print() const {
            std::cout << "CPA Result:\n";
            std::cout << "  Miss distance:    " << miss_distance << " m\n";
            std::cout << "  Time to CPA:      " << time_to_cpa << " s\n";
            std::cout << "  Missile position: " << missile_position << " m\n";
            std::cout << "  Target position:  " << target_position << " m\n";
            std::cout << "  Closing velocity: " << closing_velocity << " m/s\n";
            std::cout << "  Intercepted:       " << (intercepted ? "YES" : "NO") << "\n";
        }
    };

    /**
     * @brief Calculate CPA for current state and target
     *
     * @param missile_state Current missile state
     * @param target Current target state
     * @param max_lookahead Maximum time to look ahead [s]
     * @return CPA analysis result
     */
    static Result calculate_cpa(const State6DOF& missile_state,
                                  const Target& target,
                                  double max_lookahead = 30.0) {
        Result result;

        Vec3 rel_pos = target.position - missile_state.get_position();
        Vec3 rel_vel = target.velocity - missile_state.get_velocity();

        // Constant velocity model: r(t) = r0 + v_rel * t
        // Distance squared: |r(t)|² = |r0 + v_rel*t|²
        // Minimum occurs when derivative = 0
        //
        // d/dt |r0 + v*t|² = 2*(r0 + v*t) · v = 0
        // t_cpa = -r0·v / |v|²

        double v_rel_squared = rel_vel.magnitude_squared();

        if (v_rel_squared < 1e-12) {
            // No relative motion - already at CPA
            result.miss_distance = rel_pos.magnitude();
            result.time_to_cpa = 0.0;
            result.missile_position = missile_state.get_position();
            result.target_position = target.position;
            result.closing_velocity = 0.0;
            result.intercepted = (result.miss_distance < 1.0);
            return result;
        }

        // Time to CPA
        double t_cpa = -rel_pos.dot(rel_vel) / v_rel_squared;

        // Clamp to reasonable range
        t_cpa = std::max(0.0, std::min(t_cpa, max_lookahead));
        result.time_to_cpa = t_cpa;

        // Position at CPA
        Vec3 missile_cpa = missile_state.get_position() +
                           missile_state.get_velocity() * t_cpa;
        Vec3 target_cpa = target.position + target.velocity * t_cpa;

        result.missile_position = missile_cpa;
        result.target_position = target_cpa;

        // Miss distance
        Vec3 cpa_separation = target_cpa - missile_cpa;
        result.miss_distance = cpa_separation.magnitude();

        // Closing velocity at CPA
        result.closing_velocity = rel_vel.magnitude();

        // Check for intercept (within threshold)
        const double intercept_threshold = 5.0; // meters
        result.intercepted = (result.miss_distance < intercept_threshold);

        return result;
    }

    /**
     * @brief Calculate CPA with acceleration (for maneuvering targets)
     *
     * Uses quadratic model: r(t) = r0 + v0*t + 0.5*a*t²
     *
     * @param missile_state Current missile state
     * @param target Current target state
     * @param max_lookahead Maximum time to look ahead [s]
     * @return CPA analysis result
     */
    static Result calculate_cpa_with_accel(const State6DOF& missile_state,
                                             const Target& target,
                                             double max_lookahead = 30.0) {
        Result result;

        Vec3 rel_pos = target.position - missile_state.get_position();
        Vec3 rel_vel = target.velocity - missile_state.get_velocity();
        Vec3 rel_acc = target.acceleration - Vec3(0.0, 0.0, 0.0); // Assume no missile accel for now

        // Accelerated model: |r(t)|² = |r0 + v0*t + 0.5*a*t²|²
        // Find t that minimizes distance
        // This requires solving: (r0 + v*t + 0.5*a*t²) · (v + a*t) = 0
        // Which is a quadratic equation: At² + Bt + C = 0

        // Numerical search for minimum (more robust)
        double min_dist = rel_pos.magnitude();
        double min_time = 0.0;

        for (double t = 0.0; t <= max_lookahead; t += 0.1) {
            Vec3 pred_pos = rel_pos + rel_vel * t + rel_acc * (0.5 * t * t);
            double dist = pred_pos.magnitude();
            if (dist < min_dist) {
                min_dist = dist;
                min_time = t;
            }
        }

        result.time_to_cpa = min_time;
        result.miss_distance = min_dist;

        // Calculate positions at CPA
        Vec3 missile_cpa = missile_state.get_position() +
                           missile_state.get_velocity() * min_time;
        Vec3 target_cpa = target.position +
                           target.velocity * min_time +
                           target.acceleration * (0.5 * min_time * min_time);

        result.missile_position = missile_cpa;
        result.target_position = target_cpa;

        // Relative velocity at CPA
        Vec3 vel_at_cpa = (target.velocity + target.acceleration * min_time) -
                          missile_state.get_velocity();
        result.closing_velocity = vel_at_cpa.magnitude();

        result.intercepted = (result.miss_distance < 5.0);

        return result;
    }

    /**
     * @brief Calculate miss distance for trajectory
     *
     * Analyzes a complete trajectory to find the minimum distance.
     * Useful for post-flight analysis.
     *
     * @param trajectory_points Array of (time, missile_pos, target_pos)
     * @return CPA result with actual minimum distance
     */
    static Result analyze_trajectory(const std::vector<std::tuple<double, Vec3, Vec3>>& trajectory_points) {
        Result result;
        result.time_to_cpa = 0.0;
        result.miss_distance = std::numeric_limits<double>::max();

        if (trajectory_points.empty()) {
            return result;
        }

        for (const auto& point : trajectory_points) {
            double t = std::get<0>(point);
            Vec3 missile_pos = std::get<1>(point);
            Vec3 target_pos = std::get<2>(point);

            double dist = (target_pos - missile_pos).magnitude();

            if (dist < result.miss_distance) {
                result.miss_distance = dist;
                result.time_to_cpa = t;
                result.missile_position = missile_pos;
                result.target_position = target_pos;
            }
        }

        // Calculate closing velocity at CPA
        if (trajectory_points.size() > 1) {
            // Find adjacent points around CPA
            auto it = std::min_element(trajectory_points.begin(), trajectory_points.end(),
                [&result](const auto& a, const auto& b) {
                    double dist_a = (std::get<2>(a) - std::get<1>(a)).magnitude();
                    double dist_b = (std::get<2>(b) - std::get<1>(b)).magnitude();
                    return std::abs(dist_a - result.miss_distance) < std::abs(dist_b - result.miss_distance);
                });

            if (it != trajectory_points.end()) {
                size_t idx = std::distance(trajectory_points.begin(), it);
                if (idx > 0 && idx < trajectory_points.size() - 1) {
                    Vec3 vel1 = std::get<1>(*(it)) - std::get<1>(*(it - 1));
                    Vec3 vel2 = std::get<2>(*(it + 1)) - std::get<2>(*it);
                    double dt = std::get<0>(*(it + 1)) - std::get<0>(*it);
                    result.closing_velocity = ((vel1 + vel2) * 0.5).magnitude() / dt;
                }
            }
        }

        result.intercepted = (result.miss_distance < 5.0);

        return result;
    }

    /**
     * @brief Evaluate guidance quality based on miss distance
     *
     * @param miss_distance Minimum achieved distance [m]
     * @return Quality rating (0-100, higher is better)
     */
    static int evaluate_quality(double miss_distance) {
        if (miss_distance < 1.0) return 100;      // Direct hit
        if (miss_distance < 5.0) return 95;       // Proximity fuse
        if (miss_distance < 10.0) return 85;      // Very close
        if (miss_distance < 25.0) return 70;      // Good guidance
        if (miss_distance < 50.0) return 50;      // Fair
        if (miss_distance < 100.0) return 30;     // Poor
        if (miss_distance < 500.0) return 10;     // Very poor
        return 0;                               // Complete miss
    }

    /**
     * @brief Get quality description string
     *
     * @param miss_distance Minimum achieved distance [m]
     * @return Descriptive quality rating
     */
    static std::string get_quality_description(double miss_distance) {
        int quality = evaluate_quality(miss_distance);

        switch (quality) {
            case 100: return "PERFECT - Direct hit";
            case 95:  return "EXCELLENT - Proximity fuse intercept";
            case 85:  return "VERY GOOD - Very close approach";
            case 70:  return "GOOD - Well guided";
            case 50: return "FAIR - Acceptable guidance";
            case 30: return "POOR - Marginal guidance";
            case 10: return "VERY POOR - Significant miss";
            default:  return "FAIL - Complete miss";
        }
    }
};

/**
 * @brief Miss distance calculator for real-time guidance monitoring
 *
 * Provides continuous CPA updates during flight.
 */
class MissDistanceTracker {
public:
    MissDistanceTracker() : min_distance_(std::numeric_limits<double>::max()),
                             time_at_min_(0.0), total_time_(0.0) {}

    /**
     * @brief Update tracker with new state
     *
     * @param missile_pos Current missile position
     * @param target_pos Current target position
     * @param time Current simulation time
     */
    void update(const Vec3& missile_pos, const Vec3& target_pos, double time) {
        double dist = (target_pos - missile_pos).magnitude();

        if (dist < min_distance_) {
            min_distance_ = dist;
            time_at_min_ = time;
        }

        total_time_ = time;
    }

    /**
     * @brief Get final CPA result
     */
    MissDistance::Result get_result() const {
        MissDistance::Result result;
        result.miss_distance = min_distance_;
        result.time_to_cpa = time_at_min_;
        result.intercepted = (min_distance_ < 5.0);
        return result;
    }

    /**
     * @brief Reset tracker for new engagement
     */
    void reset() {
        min_distance_ = std::numeric_limits<double>::max();
        time_at_min_ = 0.0;
        total_time_ = 0.0;
    }

    /**
     * @brief Get current minimum distance
     */
    double get_min_distance() const { return min_distance_; }

private:
    double min_distance_;
    double time_at_min_;
    double total_time_;
};

} // namespace ballistx

#endif // BALLISTX_MISS_DISTANCE_H
