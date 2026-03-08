#include "guidance/guidance.h"
#include "guidance/proportional_navigation.h"
#include <algorithm>
#include <cctype>

namespace ballistx {

GuidancePtr GuidanceFactory::create(const std::string& type, double param) {
    // Parse type string (case-insensitive)
    std::string type_lower = type;
    std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);

    if (type_lower == "pn" || type_lower == "proportional_navigation" || type_lower == "proportional-navigation") {
        // param is navigation gain (default 3.0)
        double gain = (param > 0.0) ? param : 3.0;
        return std::make_unique<ProportionalNavigation>(gain);
    }

    if (type_lower == "pure_pursuit" || type_lower == "pure-pursuit" || type_lower == "pursuit") {
        // param is max acceleration (default 1000.0)
        double max_accel = (param > 0.0) ? param : 1000.0;
        return std::make_unique<PurePursuit>(max_accel);
    }

    if (type_lower == "true_pn" || type_lower == "true-pn" || type_lower == "true-proportional-navigation") {
        double gain = (param > 0.0) ? param : 3.0;
        return std::make_unique<ProportionalNavigation>(gain, ProportionalNavigation::Variant::TRUE_PN);
    }

    if (type_lower == "augmented_pn" || type_lower == "augmented-pn" || type_lower == "apn") {
        double gain = (param > 0.0) ? param : 3.0;
        return std::make_unique<ProportionalNavigation>(gain, ProportionalNavigation::Variant::AUGMENTED_PN);
    }

    // Unknown type - return pure PN as default
    return std::make_unique<ProportionalNavigation>(3.0);
}

} // namespace ballistx
