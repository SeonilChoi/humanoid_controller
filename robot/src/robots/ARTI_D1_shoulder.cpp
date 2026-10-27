#include <algorithm>
#include <cmath>
#include <string>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "robot/robots/ARTI_D1_shoulder.hpp"

namespace {

// Joint axes (unit vectors in the trunk frame at the URDF zero pose, shoulder_ik.py AXES). All of them pass through
// one point, so only their directions matter.
struct Axes {
    Eigen::Vector3d m7;
    Eigen::Vector3d m8;
    Eigen::Vector3d crank_coupler;
    Eigen::Vector3d coupler_plate;
    Eigen::Vector3d m8crank_plate;
};

const Axes& axes(robot::arti_d1_shoulder::Side side) {
    static const Axes left{
        Eigen::Vector3d(0.939692621, -0.342020143, 0.0),
        Eigen::Vector3d(-0.258819045, 0.0, 0.965925826),
        Eigen::Vector3d(0.26200263, 0.71984631, -0.64278761),
        Eigen::Vector3d(-0.187014701, 0.691300132, 0.697946723),
        Eigen::Vector3d(0.965925826, 0.0, 0.258819045),
    };
    static const Axes right{
        Eigen::Vector3d(0.939692621, 0.342020143, 0.0),
        Eigen::Vector3d(0.258819045, 0.0, -0.965925826),
        Eigen::Vector3d(-0.26200263, 0.71984631, 0.64278761),
        Eigen::Vector3d(0.187014297, 0.691300106, -0.697946858),
        Eigen::Vector3d(-0.965925826, 0.0, -0.258819045),
    };
    return side == robot::arti_d1_shoulder::Side::Left ? left : right;
}

Eigen::Vector3d rotate(const Eigen::Vector3d& v, const Eigen::Vector3d& n, double t) {
    return Eigen::AngleAxisd(t, n) * v;
}

constexpr double DIFF_STEP = 1.0e-6;

} // namespace

double robot::arti_d1_shoulder::m7_from(Side side, double q_m8, double q_plate, double prev) {
    const Axes& ax = axes(side);
    const Eigen::Vector3d& d7 = ax.m7;
    const Eigen::Vector3d& a = ax.crank_coupler;
    const double k = a.dot(ax.coupler_plate);   // the coupler keeps this angle between its two axes
    const Eigen::Vector3d n2 = rotate(ax.m8crank_plate, ax.m8, q_m8);
    const Eigen::Vector3d b2 = rotate(rotate(ax.coupler_plate, ax.m8, q_m8), n2, q_plate);   // plate's coupler axis
    // a rotated about d7 by t: A cos t + B sin t + C = k
    const double A = a.dot(b2) - d7.dot(a) * d7.dot(b2);
    const double B = d7.cross(a).dot(b2);
    const double C = d7.dot(a) * d7.dot(b2);
    const double q = (k - C) / std::hypot(A, B);
    if (std::abs(q) > 1.0) {
        throw std::runtime_error(std::string("[arti_d1_shoulder::m7_from] ") + (side == Side::Left ? "left" : "right") +
                                 ": loop cannot close at m8=" + std::to_string(q_m8) + ", plate=" + std::to_string(q_plate));
    }
    const double base = std::atan2(B, A);
    double roots[2];
    for (int i = 0; i < 2; ++i) {
        const double t = base + (i == 0 ? 1.0 : -1.0) * std::acos(q);
        roots[i] = std::atan2(std::sin(t), std::cos(t));
    }
    const double lo = M7_LIMITS[static_cast<int>(side)][0];
    const double hi = M7_LIMITS[static_cast<int>(side)][1];
    const auto inside = [&](double r) { return lo - 1e-6 <= r && r <= hi + 1e-6; };
    const bool in0 = inside(roots[0]);
    const bool in1 = inside(roots[1]);
    if (in0 != in1) return in0 ? roots[0] : roots[1];
    return std::abs(roots[0] - prev) <= std::abs(roots[1] - prev) ? roots[0] : roots[1];
}

double robot::arti_d1_shoulder::plate_from(Side side, double q_m8, double m7) {
    q_m8 = std::clamp(q_m8, M8_MIN, M8_MAX);
    double lo = PLATE_MIN;
    double hi = PLATE_MAX;
    const double f_lo = m7_from(side, q_m8, lo, m7);
    const double f_hi = m7_from(side, q_m8, hi, m7);
    const bool increasing = f_hi > f_lo;   // left: m7 grows with the plate angle, right: it falls
    if (increasing ? m7 <= f_lo : m7 >= f_lo) return lo;
    if (increasing ? m7 >= f_hi : m7 <= f_hi) return hi;
    // bisection: 50 halvings of the 80 deg range leave < 1e-15 rad
    for (int i = 0; i < 50; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double f = m7_from(side, q_m8, mid, m7);
        if ((f < m7) == increasing) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

double robot::arti_d1_shoulder::plate_velocity(Side side, double q_m8, double q_plate, double m8_velocity,
                                               double m7_velocity) {
    q_m8 = std::clamp(q_m8, M8_MIN + DIFF_STEP, M8_MAX - DIFF_STEP);
    q_plate = std::clamp(q_plate, PLATE_MIN + DIFF_STEP, PLATE_MAX - DIFF_STEP);
    const double m7 = m7_from(side, q_m8, q_plate);
    // m7_dot = dm7/dm8 * m8_dot + dm7/dplate * plate_dot (central differences)
    const double dm7_dm8 = (m7_from(side, q_m8 + DIFF_STEP, q_plate, m7) -
                            m7_from(side, q_m8 - DIFF_STEP, q_plate, m7)) / (2.0 * DIFF_STEP);
    const double dm7_dplate = (m7_from(side, q_m8, q_plate + DIFF_STEP, m7) -
                               m7_from(side, q_m8, q_plate - DIFF_STEP, m7)) / (2.0 * DIFF_STEP);
    return (m7_velocity - dm7_dm8 * m8_velocity) / dm7_dplate;
}
