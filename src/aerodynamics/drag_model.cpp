#include "aerodynamics/drag_model.h"
#include <stdexcept>

namespace ballistx {

DragModel::DragModel(const std::vector<DragPoint>& points) {
    set_curve(points);
}

void DragModel::add_point(double mach, double cd) {
    drag_curve_.push_back({mach, cd});

    // Keep sorted by Mach number
    std::sort(drag_curve_.begin(), drag_curve_.end(),
        [](const DragPoint& a, const DragPoint& b) {
            return a.mach < b.mach;
        });
}

void DragModel::set_curve(const std::vector<DragPoint>& points) {
    drag_curve_ = points;

    // Sort by Mach number
    std::sort(drag_curve_.begin(), drag_curve_.end(),
        [](const DragPoint& a, const DragPoint& b) {
            return a.mach < b.mach;
        });

    // Validate
    if (drag_curve_.empty()) {
        throw std::invalid_argument("Drag curve cannot be empty");
    }
}

double DragModel::get_drag_coefficient(double mach) const {
    if (drag_curve_.empty()) {
        return 0.295;  // Default fallback
    }

    // Clamp to range
    if (mach <= drag_curve_.front().mach) {
        return drag_curve_.front().cd;
    }
    if (mach >= drag_curve_.back().mach) {
        return drag_curve_.back().cd;
    }

    // Find interval and interpolate
    size_t idx = find_interval(mach);

    const auto& p1 = drag_curve_[idx];
    const auto& p2 = drag_curve_[idx + 1];

    // Linear interpolation
    double fraction = (mach - p1.mach) / (p2.mach - p1.mach);
    return p1.cd + fraction * (p2.cd - p1.cd);
}

size_t DragModel::find_interval(double mach) const {
    // Binary search for the interval containing mach
    auto it = std::upper_bound(drag_curve_.begin(), drag_curve_.end(), mach,
        [](double m, const DragPoint& p) {
            return m < p.mach;
        });

    if (it == drag_curve_.begin()) {
        return 0;
    }

    size_t idx = std::distance(drag_curve_.begin(), it) - 1;

    if (idx >= drag_curve_.size() - 1) {
        return drag_curve_.size() - 2;
    }

    return idx;
}

// Standard artillery drag curve (similar to 155mm projectile)
DragModel DragModel::standard_artillery() {
    // Mach number vs Cd data for typical artillery shell
    // Based on G7 drag curve approximation
    return DragModel({
        {0.0, 0.150},    // Stationary
        {0.5, 0.155},    // Subsonic
        {0.8, 0.170},    // Transonic onset
        {0.9, 0.200},    // Approaching Mach 1
        {0.95, 0.250},   // Near sonic
        {1.0, 0.350},    // Mach 1 (peak drag)
        {1.05, 0.380},   // Just supersonic
        {1.1, 0.365},    // Supersonic
        {1.2, 0.340},    // Supersonic
        {1.5, 0.300},    // Supersonic
        {2.0, 0.265},    // Supersonic
        {2.5, 0.250},    // High supersonic
        {3.0, 0.240}     // Very high supersonic
    });
}

// Streamlined projectile (missile, rocket)
DragModel DragModel::streamlined_projectile() {
    return DragModel({
        {0.0, 0.100},
        {0.8, 0.110},
        {0.95, 0.150},
        {1.0, 0.200},
        {1.1, 0.220},
        {1.5, 0.210},
        {2.0, 0.190},
        {2.5, 0.180},
        {3.0, 0.175}
    });
}

// Sphere drag curve (for comparison)
DragModel DragModel::sphere() {
    return DragModel({
        {0.0, 0.470},
        {0.5, 0.470},
        {0.8, 0.520},
        {0.95, 0.700},
        {1.0, 0.900},
        {1.1, 0.850},
        {1.5, 0.750},
        {2.0, 0.700},
        {3.0, 0.650}
    });
}

} // namespace ballistx
