#include <gtest/gtest.h>
#include "ballistics/projectile.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

TEST(ProjectileTest, DefaultConstructor) {
    Projectile p;
    EXPECT_DOUBLE_EQ(p.get_mass(), 0.0);
    EXPECT_DOUBLE_EQ(p.get_diameter(), 0.0);
    EXPECT_DOUBLE_EQ(p.get_drag_coefficient(), 0.0);
}

TEST(ProjectileTest, ParameterizedConstructor) {
    Projectile p(50.0, 0.155, 0.3);
    EXPECT_DOUBLE_EQ(p.get_mass(), 50.0);
    EXPECT_DOUBLE_EQ(p.get_diameter(), 0.155);
    EXPECT_DOUBLE_EQ(p.get_drag_coefficient(), 0.3);
}

TEST(ProjectileTest, AreaCalculation) {
    Projectile p(50.0, 0.155, 0.3);
    double area = p.get_area();

    // Area = π * r² = π * (0.155/2)²
    double expected = M_PI * std::pow(0.155 / 2.0, 2);
    EXPECT_NEAR(area, expected, 1e-6);
}

TEST(ProjectileTest, SettersAndGetters) {
    Projectile p;
    p.set_mass(100.0);
    p.set_diameter(0.120);
    p.set_drag_coefficient(0.25);

    EXPECT_DOUBLE_EQ(p.get_mass(), 100.0);
    EXPECT_DOUBLE_EQ(p.get_diameter(), 0.120);
    EXPECT_DOUBLE_EQ(p.get_drag_coefficient(), 0.25);
}

TEST(ProjectileTest, MachNumber) {
    // get_mach_number uses speed of sound from atmosphere or defaults
    // It requires a valid projectile (mass > 0, diameter > 0)
    Projectile p(10.0, 0.01, 0.1);

    // Using default sound speed of 343 m/s if no atmosphere specified
    double mach1 = p.get_mach_number(340.0);
    EXPECT_NEAR(mach1, 0.99, 0.01);  // Allow some tolerance

    double mach2 = p.get_mach_number(686.0);
    EXPECT_NEAR(mach2, 2.0, 0.01);
}

TEST(ProjectileTest, IsValid) {
    Projectile p(10.0, 0.01, 0.1);  // Valid projectile
    EXPECT_TRUE(p.is_valid());

    Projectile p2;  // Empty projectile
    EXPECT_FALSE(p2.is_valid());
}

TEST(ProjectileTest, CreateStandardProjectile) {
    auto p = Projectile::create(ProjectileType::ARTILLERY_155MM);

    EXPECT_TRUE(p.is_valid());
    EXPECT_GT(p.get_mass(), 0.0);
    EXPECT_GT(p.get_diameter(), 0.0);
    EXPECT_GT(p.get_drag_coefficient(), 0.0);
}

TEST(ProjectileTest, GetName) {
    auto p = Projectile::create(ProjectileType::ARTILLERY_155MM);
    std::string name = p.get_name();

    EXPECT_FALSE(name.empty());
}

TEST(ProjectileTest, IsCustom) {
    auto standard = Projectile::create(ProjectileType::ARTILLERY_155MM);
    EXPECT_FALSE(standard.is_custom());

    Projectile custom(10.0, 0.01, 0.1);
    EXPECT_TRUE(custom.is_custom());
}
