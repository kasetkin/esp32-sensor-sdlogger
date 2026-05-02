#include "unity.h"
#include "gpstask.h"

// All expected values derived from R = 6371000.0 m (Haversine mean Earth radius).
// Verified analytically and with Python before being hard-coded here.

static void test_geo_distance_same_point(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, GpsTask::geoDistance(0.0, 0.0, 0.0, 0.0));
}

static void test_geo_distance_1deg_latitude(void)
{
    // Pure north: d = R * π/180 = 111194.9266 m
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 111194.9266, GpsTask::geoDistance(0.0, 0.0, 1.0, 0.0));
}

static void test_geo_distance_1deg_longitude_at_equator(void)
{
    // Pure east at equator: cos(0)=1, same formula as 1° latitude
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 111194.9266, GpsTask::geoDistance(0.0, 0.0, 0.0, 1.0));
}

static void test_geo_distance_quarter_circumference(void)
{
    // 90° along equator: d = R * π/2 = 10007543.398 m
    TEST_ASSERT_DOUBLE_WITHIN(100.0, 10007543.398, GpsTask::geoDistance(0.0, 0.0, 0.0, 90.0));
}

static void test_geo_distance_antipodal(void)
{
    // Half of Earth circumference: d = R * π = 20015086.796 m
    TEST_ASSERT_DOUBLE_WITHIN(100.0, 20015086.796, GpsTask::geoDistance(0.0, 0.0, 0.0, 180.0));
}

static void test_geo_distance_antimeridian(void)
{
    // lon=-179 to lon=+179: shortest path is 2° across the antimeridian.
    // Haversine handles this correctly via sin(π-θ)=sin(θ) — no normalization needed.
    // Expected: same as (0,0)→(0,2) = 222389.853 m
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 222389.853, GpsTask::geoDistance(0.0, -179.0, 0.0, 179.0));
}

static void test_geo_distance_near_south_pole_antimeridian(void)
{
    // Near south pole: tiny circle even though longitudes span ±179°
    // Expected: cos(89°) * R * π/180 * 2 ≈ 3881.041 m
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 3881.041, GpsTask::geoDistance(-89.0, -179.0, -89.0, 179.0));
}

static void test_geo_distance_symmetric(void)
{
    const double ab = GpsTask::geoDistance(55.75, 37.62, 48.86, 2.35);
    const double ba = GpsTask::geoDistance(48.86, 2.35, 55.75, 37.62);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, ab, ba);
}

void run_geodistance_tests(void)
{
    RUN_TEST(test_geo_distance_same_point);
    RUN_TEST(test_geo_distance_1deg_latitude);
    RUN_TEST(test_geo_distance_1deg_longitude_at_equator);
    RUN_TEST(test_geo_distance_quarter_circumference);
    RUN_TEST(test_geo_distance_antipodal);
    RUN_TEST(test_geo_distance_antimeridian);
    RUN_TEST(test_geo_distance_near_south_pole_antimeridian);
    RUN_TEST(test_geo_distance_symmetric);
}
