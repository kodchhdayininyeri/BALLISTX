#ifndef BALLISTX_DRAG_MODEL_H
#define BALLISTX_DRAG_MODEL_H

#include <vector>
#include <algorithm>
#include <cmath>

namespace ballistx {

/**
 * @brief Mach-dependent drag coefficient model
 *
 * Drag coefficient varies significantly with Mach number:
 * - Subsonic (M < 0.8): Nearly constant Cd
 * - Transonic (0.8 < M < 1.2): Sharp rise in drag
 * - Supersonic (M > 1.2): Cd decreases with speed
 *
 * Uses linear interpolation between data points.
 */
class DragModel {
public:
    struct DragPoint {
        double mach;     // Mach number
        double cd;       // Drag coefficient
    };

    // Constructors
    DragModel() = default;
    explicit DragModel(const std::vector<DragPoint>& points);

    // Get drag coefficient for given Mach number
    double get_drag_coefficient(double mach) const;

    // Add data point
    void add_point(double mach, double cd);

    // Set all data points at once
    void set_curve(const std::vector<DragPoint>& points);

    // Get default drag curve for standard artillery projectile
    static DragModel standard_artillery();
    static DragModel streamlined_projectile();
    static DragModel sphere();

    // Check if model is valid
    bool is_valid() const { return !drag_curve_.empty(); }

    // Get curve data
    const std::vector<DragPoint>& get_curve() const { return drag_curve_; }

private:
    std::vector<DragPoint> drag_curve_;

    // Helper: binary search for interpolation interval
    size_t find_interval(double mach) const;
};

} // namespace ballistx

#endif // BALLISTX_DRAG_MODEL_H
