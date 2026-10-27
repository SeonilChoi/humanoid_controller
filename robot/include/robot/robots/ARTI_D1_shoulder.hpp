#ifndef ROBOT_ROBOTS_ARTI_D1_SHOULDER_HPP_
#define ROBOT_ROBOTS_ARTI_D1_SHOULDER_HPP_

// ARTI-D1 shoulder: mapping between the URDF (serial) model and the real 2-DoF 5-bar shoulder linkage (as in "Olaf:
// Bringing an Animated Character to Life in the Physical World"). Port of pocketbot_final_v5 shoulder_ik.py.
//
// The URDF / policy drives each shoulder with
//   {side}_arm_m8             = real m8 servo angle (same joint, same axis and sign)
//   {side}_arm_m8crank_plate  = passive plate joint of the real loop
// The real m7 servo (the Dynamixel on the {side}_arm_m8crank_plate motor ID) follows from those two. All angles are
// rad, 0 = CAD pose, signs about the Fusion joint axes (= the URDF axes).

namespace robot::arti_d1_shoulder {

enum class Side { Left = 0, Right = 1 };

// URDF {side}_arm_m8crank_plate limits (since v5), the range plate_from() solves in.
constexpr double PLATE_MIN = -0.226893;
constexpr double PLATE_MAX = 1.169371;

// URDF {side}_arm_m8 limits; plate_from() clamps m8 into them (the loop closes everywhere inside).
constexpr double M8_MIN = -1.047198;
constexpr double M8_MAX = 0.349066;

// Real m7 limits; the URDF limits keep m7 inside them.
constexpr double M7_LIMITS[2][2] = {{-0.349066, 1.308997}, {-1.308997, 0.349066}};

// Real m7 angle for URDF (m8, m8crank_plate) angles. Picks the loop branch inside M7_LIMITS, nearest prev.
// Throws std::runtime_error when the loop cannot close.
double m7_from(Side side, double q_m8, double q_plate, double prev = 0.0);

// Inverse of m7_from: URDF m8crank_plate angle for real (m8, m7) angles, clamped to [PLATE_MIN, PLATE_MAX] (m7 is
// monotonic in the plate angle over the URDF range).
double plate_from(Side side, double q_m8, double m7);

// URDF m8crank_plate velocity for the real m8 / m7 velocities at the URDF pose (q_m8, q_plate).
double plate_velocity(Side side, double q_m8, double q_plate, double m8_velocity, double m7_velocity);

} // namespace robot::arti_d1_shoulder

#endif // ROBOT_ROBOTS_ARTI_D1_SHOULDER_HPP_
