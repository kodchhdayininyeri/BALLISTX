#include "ballistics/projectile.h"
#include <stdexcept>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

Projectile::Projectile(double mass, double diameter, double drag_coefficient)
    : mass_(mass), diameter_(diameter), drag_coefficient_(drag_coefficient),
      type_(ProjectileType::CUSTOM) {
    if (diameter_ > 0.0) {
        area_ = M_PI * diameter_ * diameter_ / 4.0;
    }
}

void Projectile::set_diameter(double diameter) {
    diameter_ = diameter;
    if (diameter_ > 0.0) {
        area_ = M_PI * diameter_ * diameter_ / 4.0;
    } else {
        area_ = 0.0;
    }
}

double Projectile::get_mach_number(double velocity, double speed_of_sound) const {
    if (speed_of_sound <= 0.0) {
        return 0.0;
    }
    return velocity / speed_of_sound;
}

bool Projectile::is_valid() const {
    return mass_ > 0.0 && diameter_ > 0.0 && drag_coefficient_ >= 0.0;
}

std::string Projectile::get_name() const {
    if (!custom_name_.empty()) {
        return custom_name_;
    }
    return get_type_name(type_);
}

// ============================================================================
// Factory Methods
// ============================================================================

Projectile Projectile::create(ProjectileType type) {
    ProjectileProperties props = get_properties(type);
    Projectile proj(props.mass, props.diameter, props.drag_coefficient);
    proj.type_ = type;
    proj.custom_name_ = props.name;
    return proj;
}

Projectile Projectile::create(const ProjectileProperties& props) {
    Projectile proj(props.mass, props.diameter, props.drag_coefficient);
    proj.type_ = ProjectileType::CUSTOM;
    proj.custom_name_ = props.name;
    return proj;
}

// ============================================================================
// Projectile Properties Database
// ============================================================================

ProjectileProperties Projectile::get_properties(ProjectileType type) {
    switch (type) {
        // ==================== ARTILLERY SHELLS ====================
        case ProjectileType::ARTILLERY_155MM: {
            // M107 155mm HE projectile (standard NATO)
            // Mass: 43.2 kg, Length: 60-80 cm
            ProjectileProperties props;
            props.mass = 43.2;              // kg
            props.diameter = 0.155;         // m (155mm)
            props.drag_coefficient = 0.25;  // Streamlined
            props.name = "155mm HE Artillery";
            props.description = "Standard NATO 155mm high-explosive projectile (M107)";
            return props;
        }

        case ProjectileType::ARTILLERY_105MM: {
            // M1 105mm HE projectile
            ProjectileProperties props;
            props.mass = 15.1;              // kg
            props.diameter = 0.105;         // m (105mm)
            props.drag_coefficient = 0.28;
            props.name = "105mm HE Artillery";
            props.description = "Light artillery 105mm high-explosive projectile";
            return props;
        }

        case ProjectileType::ARTILLERY_120MM_MORTAR: {
            // 120mm mortar bomb
            ProjectileProperties props;
            props.mass = 13.2;              // kg
            props.diameter = 0.120;         // m (120mm)
            props.drag_coefficient = 0.35;  // Less streamlined
            props.name = "120mm Mortar";
            props.description = "Standard 120mm mortar bomb";
            return props;
        }

        case ProjectileType::ARTILLERY_81MM_MORTAR: {
            // 81mm mortar bomb
            ProjectileProperties props;
            props.mass = 4.2;               // kg
            props.diameter = 0.081;         // m (81mm)
            props.drag_coefficient = 0.38;
            props.name = "81mm Mortar";
            props.description = "Standard 81mm mortar bomb";
            return props;
        }

        case ProjectileType::ARTILLERY_60MM_MORTAR: {
            // 60mm mortar bomb
            ProjectileProperties props;
            props.mass = 1.7;               // kg
            props.diameter = 0.060;         // m (60mm)
            props.drag_coefficient = 0.40;
            props.name = "60mm Mortar";
            props.description = "Light 60mm mortar bomb";
            return props;
        }

        // ==================== ROCKETS ====================
        case ProjectileType::ROCKET_70MM: {
            // Hydra 70 unguided rocket
            ProjectileProperties props;
            props.mass = 6.2;               // kg (warhead + motor)
            props.diameter = 0.070;         // m (70mm / 2.75 inch)
            props.drag_coefficient = 0.32;
            props.name = "70mm Hydra Rocket";
            props.description = "Unguided 70mm rocket (Hydra 70)";
            return props;
        }

        case ProjectileType::ROCKET_122MM: {
            // Grad 122mm rocket
            ProjectileProperties props;
            props.mass = 66.0;              // kg
            props.diameter = 0.122;         // m (122mm)
            props.drag_coefficient = 0.30;
            props.name = "122mm Grad Rocket";
            props.description = "Soviet 122mm artillery rocket (BM-21 Grad)";
            return props;
        }

        case ProjectileType::ROCKET_107MM: {
            // 107mm rocket (Type 63)
            ProjectileProperties props;
            props.mass = 18.8;              // kg
            props.diameter = 0.107;         // m (107mm)
            props.drag_coefficient = 0.32;
            props.name = "107mm Rocket";
            props.description = "107mm artillery rocket";
            return props;
        }

        case ProjectileType::ROCKET_57MM: {
            // S-5 57mm rocket
            ProjectileProperties props;
            props.mass = 4.0;               // kg
            props.diameter = 0.057;         // m (57mm)
            props.drag_coefficient = 0.35;
            props.name = "57mm S-5 Rocket";
            props.description = "Light 57mm unguided rocket";
            return props;
        }

        // ==================== MISSILES ====================
        case ProjectileType::MISSILE_AIR_TO_AIR: {
            // AMRAAM-like medium-range AAM
            ProjectileProperties props;
            props.mass = 161.0;             // kg (AIM-120C)
            props.diameter = 0.178;         // m (178mm / 7 inch)
            props.drag_coefficient = 0.18;  // Very streamlined
            props.name = "Medium-Range AAM";
            props.description = "Beyond-visual-range air-to-air missile (AMRAAM-type)";
            return props;
        }

        case ProjectileType::MISSILE_AIR_TO_GROUND: {
            // AGM-65 Maverick
            ProjectileProperties props;
            props.mass = 136.0;             // kg (AGM-65H)
            props.diameter = 0.305;         // m (305mm / 12 inch)
            props.drag_coefficient = 0.22;
            props.name = "AGM Maverick";
            props.description = "Air-to-ground missile (Maverick-type)";
            return props;
        }

        case ProjectileType::MISSILE_SURFACE_TO_AIR: {
            // MIM-104 Patriot
            ProjectileProperties props;
            props.mass = 900.0;             // kg (PAC-2)
            props.diameter = 0.410;         // m (410mm)
            props.drag_coefficient = 0.20;
            props.name = "SAM Patriot";
            props.description = "Surface-to-air missile (Patriot-type)";
            return props;
        }

        case ProjectileType::MISSILE_ANTI_TANK: {
            // FGM-148 Javelin
            ProjectileProperties props;
            props.mass = 11.8;              // kg (missile only)
            props.diameter = 0.127;         // m (127mm)
            props.drag_coefficient = 0.28;
            props.name = "ATGM Javelin";
            props.description = "Anti-tank guided missile (Javelin-type)";
            return props;
        }

        // ==================== SMALL ARMS ====================
        case ProjectileType::BULLET_5_56MM: {
            // 5.56x45mm NATO (M855)
            ProjectileProperties props;
            props.mass = 0.004;             // kg (4g / 62 grain)
            props.diameter = 0.00566;       // m (5.66mm)
            props.drag_coefficient = 0.29;  // G7 BC ~0.15
            props.name = "5.56x45mm NATO";
            props.description = "Standard rifle cartridge (M855/SS109)";
            return props;
        }

        case ProjectileType::BULLET_7_62MM: {
            // 7.62x51mm NATO (M80)
            ProjectileProperties props;
            props.mass = 0.0096;            // kg (9.6g / 148 grain)
            props.diameter = 0.00782;       // m (7.82mm)
            props.drag_coefficient = 0.28;
            props.name = "7.62x51mm NATO";
            props.description = "Standard rifle cartridge (M80)";
            return props;
        }

        case ProjectileType::BULLET_12_7MM: {
            // 12.7x99mm (.50 BMG)
            ProjectileProperties props;
            props.mass = 0.046;             // kg (46g / 715 grain)
            props.diameter = 0.0127;        // m (12.7mm / .50 inch)
            props.drag_coefficient = 0.25;
            props.name = "12.7x99mm (.50 BMG)";
            props.description = "Heavy machine gun cartridge (.50 BMG/M2)";
            return props;
        }

        // ==================== TANK ROUNDS ====================
        case ProjectileType::APFSDS_120MM: {
            // M829 120mm APFSDS (tank sabot round)
            // Note: This is the penetrator mass, not full sabot
            ProjectileProperties props;
            props.mass = 8.2;               // kg (penetrator only)
            props.diameter = 0.025;         // m (25mm penetrator)
            props.drag_coefficient = 0.15;  // Very low drag after sabot discard
            props.name = "120mm APFSDS";
            props.description = "Armor-piercing fin-stabilized discarding sabot (M829)";
            return props;
        }

        case ProjectileType::HEAT_120MM: {
            // M830 120mm HEAT
            ProjectileProperties props;
            props.mass = 24.0;              // kg
            props.diameter = 0.120;         // m (120mm)
            props.drag_coefficient = 0.30;
            props.name = "120mm HEAT";
            props.description = "High-explosive anti-tank (M830)";
            return props;
        }

        // ==================== CUSTOM ====================
        case ProjectileType::CUSTOM:
        default: {
            ProjectileProperties props;
            props.mass = 1.0;
            props.diameter = 0.1;
            props.drag_coefficient = 0.3;
            props.name = "Custom Projectile";
            props.description = "User-defined projectile";
            return props;
        }
    }
}

std::string Projectile::get_type_name(ProjectileType type) {
    switch (type) {
        case ProjectileType::ARTILLERY_155MM:        return "ARTILLERY_155MM";
        case ProjectileType::ARTILLERY_105MM:        return "ARTILLERY_105MM";
        case ProjectileType::ARTILLERY_120MM_MORTAR: return "ARTILLERY_120MM_MORTAR";
        case ProjectileType::ARTILLERY_81MM_MORTAR:  return "ARTILLERY_81MM_MORTAR";
        case ProjectileType::ARTILLERY_60MM_MORTAR:  return "ARTILLERY_60MM_MORTAR";
        case ProjectileType::ROCKET_70MM:            return "ROCKET_70MM";
        case ProjectileType::ROCKET_122MM:           return "ROCKET_122MM";
        case ProjectileType::ROCKET_107MM:           return "ROCKET_107MM";
        case ProjectileType::ROCKET_57MM:            return "ROCKET_57MM";
        case ProjectileType::MISSILE_AIR_TO_AIR:     return "MISSILE_AIR_TO_AIR";
        case ProjectileType::MISSILE_AIR_TO_GROUND:  return "MISSILE_AIR_TO_GROUND";
        case ProjectileType::MISSILE_SURFACE_TO_AIR: return "MISSILE_SURFACE_TO_AIR";
        case ProjectileType::MISSILE_ANTI_TANK:      return "MISSILE_ANTI_TANK";
        case ProjectileType::BULLET_5_56MM:          return "BULLET_5_56MM";
        case ProjectileType::BULLET_7_62MM:          return "BULLET_7_62MM";
        case ProjectileType::BULLET_12_7MM:          return "BULLET_12_7MM";
        case ProjectileType::APFSDS_120MM:           return "APFSDS_120MM";
        case ProjectileType::HEAT_120MM:             return "HEAT_120MM";
        case ProjectileType::CUSTOM:                 return "CUSTOM";
        default:                                      return "UNKNOWN";
    }
}

} // namespace ballistx
