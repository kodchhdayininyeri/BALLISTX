#include "ballistics/projectile.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

Projectile::Projectile(double mass, double diameter, double drag_coefficient)
    : mass_(mass), diameter_(diameter), drag_coefficient_(drag_coefficient) {
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

} // namespace ballistx
