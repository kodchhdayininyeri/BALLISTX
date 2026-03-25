#ifndef BALLISTX_DRAG_MODEL_H
#define BALLISTX_DRAG_MODEL_H

#include <vector>
#include <algorithm>
#include <cmath>

namespace ballistx {

/**
 * @brief Mach-dependent drag coefficient model
 *
 * Models the variation of drag coefficient with Mach number for accurate
 * ballistic calculations. Drag coefficient changes dramatically across
 * the transonic region.
 *
 * **Mach Regimes:**
 * - Subsonic (M < 0.8): Nearly constant Cd
 * - Transonic (0.8 < M < 1.2): Sharp rise in drag (wave drag)
 * - Supersonic (M > 1.2): Cd decreases with speed
 *
 * **Drag Force Formula:**
 * ```
 * F_drag = ½ × ρ × v² × Cd(M) × A
 * ```
 * where ρ is air density, v is velocity, Cd(M) is Mach-dependent
 * drag coefficient, and A is cross-sectional area.
 *
 * @example
 * @code
 * // Create drag model for standard artillery shell
 * DragModel drag = DragModel::standard_artillery();
 *
 * // Get drag coefficient at Mach 1.5
 * double cd = drag.get_drag_coefficient(1.5);
 *
 * // Create custom drag curve
 * DragModel custom;
 * custom.add_point(0.0, 0.15);   // Subsonic
 * custom.add_point(1.0, 0.40);   // Sonic
 * custom.add_point(2.0, 0.25);   // Supersonic
 * custom.add_point(3.0, 0.20);   // High supersonic
 * @endcode
 *
 * @see Atmosphere for air density and speed of sound
 * @see Projectile for cross-sectional area
 */
class DragModel {
public:
    /**
     * @brief Single data point on drag curve
     *
     * Defines drag coefficient at a specific Mach number.
     */
    struct DragPoint {
        double mach;  ///< Mach number
        double cd;    ///< Drag coefficient (dimensionless)
    };

    /**
     * @brief Default constructor
     *
     * Creates empty drag model. Add points using add_point() or
     * use a factory method like standard_artillery().
     */
    DragModel() = default;

    /**
     * @brief Create drag model from data points
     *
     * Initializes model with provided drag curve data.
     * Points should be sorted by Mach number.
     *
     * @param points Vector of (mach, cd) data points
     */
    explicit DragModel(const std::vector<DragPoint>& points);

    /**
     * @brief Get drag coefficient for given Mach number
     *
     * Returns drag coefficient using linear interpolation between
     * data points. Extrapolates beyond the curve endpoints.
     *
     * @param mach Mach number
     * @return Drag coefficient at this Mach number
     *
     * @example
     * @code
     * DragModel drag = DragModel::standard_artillery();
     * double cd_1_0 = drag.get_drag_coefficient(1.0);   // At Mach 1
     * double cd_2_5 = drag.get_drag_coefficient(2.5);   // At Mach 2.5
     * @endcode
     */
    double get_drag_coefficient(double mach) const;

    /**
     * @brief Add data point to drag curve
     *
     * Inserts a new (Mach, Cd) data point. Points are automatically
     * sorted by Mach number.
     *
     * @param mach Mach number
     * @param cd Drag coefficient
     */
    void add_point(double mach, double cd);

    /**
     * @brief Set entire drag curve at once
     *
     * Replaces existing data with new curve.
     *
     * @param points Vector of drag curve points
     */
    void set_curve(const std::vector<DragPoint>& points);

    /**
     * @brief Get standard artillery drag curve
     *
     * Returns a drag model typical for 155mm artillery shells.
     * Characterized by significant drag rise at transonic speeds.
     *
     * @return DragModel for standard artillery
     */
    static DragModel standard_artillery();

    /**
     * @brief Get streamlined projectile drag curve
     *
     * Returns a drag model for low-drag streamlined projectiles
     * like boat-tailed bullets or shaped projectiles.
     *
     * @return DragModel for streamlined projectile
     */
    static DragModel streamlined_projectile();

    /**
     * @brief Get sphere drag curve
     *
     * Returns drag model for a sphere (bluff body).
     * Cd ≈ 0.47 subsonic, rises sharply at transonic.
     *
     * @return DragModel for sphere
     */
    static DragModel sphere();

    /**
     * @brief Check if model has valid data
     *
     * @return true if drag curve has data points, false otherwise
     */
    bool is_valid() const { return !drag_curve_.empty(); }

    /**
     * @brief Get drag curve data
     *
     * @return Vector of drag curve data points
     */
    const std::vector<DragPoint>& get_curve() const { return drag_curve_; }

private:
    std::vector<DragPoint> drag_curve_;  ///< Drag coefficient vs Mach curve

    /**
     * @brief Find interpolation interval for Mach number
     *
     * Uses binary search to locate the interval containing
     * the given Mach number.
     *
     * @param mach Mach number
     * @return Index of lower bound point
     */
    size_t find_interval(double mach) const;
};

} // namespace ballistx

#endif // BALLISTX_DRAG_MODEL_H
