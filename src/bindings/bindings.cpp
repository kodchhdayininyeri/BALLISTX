/**
 * @file bindings.cpp
 * @brief Python bindings for BALLISTX ballistic simulation library
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "utils/vec3.h"
#include "utils/quaternion.h"
#include "ballistics/state_6dof.h"
#include "ballistics/projectile.h"
#include "atmosphere/isa_model.h"
#include "guidance/guidance.h"
#include "guidance/proportional_navigation.h"
#include "guidance/miss_distance.h"

namespace py = pybind11;
using namespace ballistx;

PYBIND11_MODULE(_ballistx, m) {
    m.doc() = "BALLISTX - Ballistic Simulation Library Python Bindings";

    // Vec3
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<double, double, double>(),
             py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("magnitude", &Vec3::magnitude)
        .def("magnitude_squared", &Vec3::magnitude_squared)
        .def("normalized", &Vec3::normalized)
        .def("normalize", &Vec3::normalize)
        .def("dot", &Vec3::dot, py::arg("other"))
        .def("cross", &Vec3::cross, py::arg("other"))
        .def("distance_to", &Vec3::distance_to, py::arg("other"))
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * double())
        .def(double() * py::self)
        .def(py::self / double())
        .def(-py::self)
        .def("__repr__", [](const Vec3& v) {
            return "Vec3(" + std::to_string(v.x) + ", " +
                          std::to_string(v.y) + ", " +
                          std::to_string(v.z) + ")";
        })
        .def_static("zero", &Vec3::zero)
        .def_static("up", &Vec3::up)
        .def_static("forward", &Vec3::forward)
        .def_static("right", &Vec3::right);

    // Quaternion
    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<double, double, double, double>(),
             py::arg("w") = 1.0, py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
        .def("magnitude", &Quaternion::magnitude)
        .def("normalized", &Quaternion::normalized)
        .def("conjugate", &Quaternion::conjugate)
        .def("inverse", &Quaternion::inverse)
        .def("rotate", &Quaternion::rotate, py::arg("vec"))
        .def(py::self * py::self)
        .def_static("identity", &Quaternion::identity)
        .def_static("from_axis_angle", &Quaternion::from_axis_angle,
                    py::arg("axis"), py::arg("angle_rad"))
        .def_static("from_euler", &Quaternion::from_euler,
                    py::arg("roll"), py::arg("pitch"), py::arg("yaw"));

    // State6DOF
    py::class_<State6DOF>(m, "State6DOF")
        .def(py::init<const Vec3&, const Vec3&, const Quaternion&, const Vec3&>(),
             py::arg("position"), py::arg("velocity"),
             py::arg("orientation"), py::arg("angular_velocity"))
        .def("get_position", &State6DOF::get_position)
        .def("set_position", &State6DOF::set_position, py::arg("pos"))
        .def("get_velocity", &State6DOF::get_velocity)
        .def("set_velocity", &State6DOF::set_velocity, py::arg("vel"))
        .def("get_orientation", &State6DOF::get_orientation)
        .def("set_orientation", &State6DOF::set_orientation, py::arg("quat"))
        .def("get_angular_velocity", &State6DOF::get_angular_velocity)
        .def("set_angular_velocity", &State6DOF::set_angular_velocity, py::arg("ang_vel"));

    // Target
    py::class_<Target>(m, "Target")
        .def(py::init<const Vec3&, const Vec3&, const Vec3&>(),
             py::arg("position") = Vec3(),
             py::arg("velocity") = Vec3(),
             py::arg("acceleration") = Vec3())
        .def_readwrite("position", &Target::position)
        .def_readwrite("velocity", &Target::velocity)
        .def_readwrite("acceleration", &Target::acceleration)
        .def("is_moving", &Target::is_moving)
        .def("has_prediction", &Target::has_prediction)
        .def("predict_position", &Target::predict_position, py::arg("time_ahead"))
        .def("predict_velocity", &Target::predict_velocity, py::arg("time_ahead"));

    // GuidanceCommand
    py::class_<GuidanceCommand>(m, "GuidanceCommand")
        .def(py::init<>())
        .def_readwrite("acceleration_command", &GuidanceCommand::acceleration_command)
        .def_readwrite("detonation_command", &GuidanceCommand::detonation_command)
        .def_readwrite("is_valid", &GuidanceCommand::is_valid)
        .def_readwrite("time_to_go", &GuidanceCommand::time_to_go)
        .def_readwrite("status_msg", &GuidanceCommand::status_msg);

    // Guidance base class (abstract)
    py::class_<Guidance>(m, "Guidance");

    // ProportionalNavigation
    py::class_<ProportionalNavigation, Guidance>(m, "ProportionalNavigation")
        .def(py::init<double>(), py::arg("nav_gain") = 3.0)
        .def("calculate_command", &ProportionalNavigation::calculate_command,
             py::arg("state"), py::arg("target"))
        .def("reset", &ProportionalNavigation::reset)
        .def("get_name", &ProportionalNavigation::get_name)
        .def("set_max_acceleration", &ProportionalNavigation::set_max_acceleration,
             py::arg("max_accel"))
        .def("set_proximity_threshold", &ProportionalNavigation::set_proximity_threshold,
             py::arg("threshold"));

    // PurePursuit
    py::class_<PurePursuit, Guidance>(m, "PurePursuit")
        .def(py::init<double>(), py::arg("max_accel") = 300.0)
        .def("calculate_command", &PurePursuit::calculate_command,
             py::arg("state"), py::arg("target"))
        .def("get_name", &PurePursuit::get_name);

    // MissDistance::Result
    py::class_<MissDistance::Result>(m, "MissDistanceResult")
        .def(py::init<>())
        .def_readwrite("miss_distance", &MissDistance::Result::miss_distance)
        .def_readwrite("time_to_cpa", &MissDistance::Result::time_to_cpa)
        .def_readwrite("missile_position", &MissDistance::Result::missile_position)
        .def_readwrite("target_position", &MissDistance::Result::target_position)
        .def_readwrite("closing_velocity", &MissDistance::Result::closing_velocity)
        .def_readwrite("intercepted", &MissDistance::Result::intercepted);

    // MissDistance
    py::class_<MissDistance>(m, "MissDistance")
        .def_static("calculate_cpa", &MissDistance::calculate_cpa,
                    py::arg("missile_state"), py::arg("target"),
                    py::arg("max_lookahead") = 30.0)
        .def_static("calculate_cpa_with_accel", &MissDistance::calculate_cpa_with_accel,
                    py::arg("missile_state"), py::arg("target"),
                    py::arg("max_lookahead") = 30.0)
        .def_static("analyze_trajectory", &MissDistance::analyze_trajectory,
                    py::arg("trajectory_points"))
        .def_static("evaluate_quality", &MissDistance::evaluate_quality,
                    py::arg("miss_distance"))
        .def_static("get_quality_description", &MissDistance::get_quality_description,
                    py::arg("miss_distance"));

    // MissDistanceTracker
    py::class_<MissDistanceTracker>(m, "MissDistanceTracker")
        .def(py::init<>())
        .def("update", &MissDistanceTracker::update,
             py::arg("missile_pos"), py::arg("target_pos"), py::arg("time"))
        .def("get_result", &MissDistanceTracker::get_result)
        .def("reset", &MissDistanceTracker::reset)
        .def("get_min_distance", &MissDistanceTracker::get_min_distance);

    // Atmosphere
    py::class_<Atmosphere>(m, "Atmosphere")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("altitude"))
        .def("get_temperature", &Atmosphere::get_temperature)
        .def("get_pressure", &Atmosphere::get_pressure)
        .def("get_density", &Atmosphere::get_density)
        .def("get_altitude", &Atmosphere::get_altitude)
        .def("get_speed_of_sound", &Atmosphere::get_speed_of_sound)
        .def("set_altitude", &Atmosphere::set_altitude, py::arg("altitude"))
        .def("get_wind_velocity", &Atmosphere::get_wind_velocity)
        .def("set_wind_velocity", &Atmosphere::set_wind_velocity, py::arg("wind"))
        .def_static("calculate_temperature", &Atmosphere::calculate_temperature,
                    py::arg("altitude"))
        .def_static("calculate_pressure", &Atmosphere::calculate_pressure,
                    py::arg("altitude"))
        .def_static("calculate_density", &Atmosphere::calculate_density,
                    py::arg("altitude"));

    // ProjectileType enum
    py::enum_<ProjectileType>(m, "ProjectileType")
        .value("ARTILLERY_155MM", ProjectileType::ARTILLERY_155MM)
        .value("ARTILLERY_105MM", ProjectileType::ARTILLERY_105MM)
        .value("ROCKET_70MM", ProjectileType::ROCKET_70MM)
        .value("MISSILE_AIR_TO_AIR", ProjectileType::MISSILE_AIR_TO_AIR)
        .value("BULLET_5_56MM", ProjectileType::BULLET_5_56MM)
        .value("CUSTOM", ProjectileType::CUSTOM)
        .export_values();

    // Projectile
    py::class_<Projectile>(m, "Projectile")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("mass"), py::arg("diameter"), py::arg("drag_coefficient"))
        .def_static("create", py::overload_cast<ProjectileType>(&Projectile::create),
                    py::arg("type"))
        .def("get_mass", &Projectile::get_mass)
        .def("get_diameter", &Projectile::get_diameter)
        .def("get_drag_coefficient", &Projectile::get_drag_coefficient)
        .def("get_area", &Projectile::get_area)
        .def("get_cross_sectional_area", &Projectile::get_cross_sectional_area)
        .def("set_mass", &Projectile::set_mass, py::arg("mass"))
        .def("set_diameter", &Projectile::set_diameter, py::arg("diameter"))
        .def("set_drag_coefficient", &Projectile::set_drag_coefficient, py::arg("cd"))
        .def("is_valid", &Projectile::is_valid)
        .def("get_name", &Projectile::get_name)
        .def("is_custom", &Projectile::is_custom);

    m.attr("__version__") = "0.1.0";
}
