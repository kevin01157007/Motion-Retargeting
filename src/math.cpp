#include "motion.hpp"

Quat normalize(Quat q) {
    // TODO 1a: return q divided by its length. Handle a near-zero quaternion.
    const double norm = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
    if (norm <= 1e-12) return {};
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Quat multiply(Quat a, Quat b) {
    // TODO 1b: Hamilton product. Composition convention: a*b applies b, then a.
    double a_w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    double a_x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    double a_y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    double a_z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    a.w = a_w; a.x = a_x; a.y = a_y; a.z = a_z;
    return a;
}

Quat axisAngle(Vec3 axis, double radians) {
    // TODO 1c: normalize axis, use cos(theta/2) and sin(theta/2).
    const double axis_length = std::hypot(std::hypot(axis.x, axis.y), axis.z);
    Vec3 normalized_axis{};
    if (axis_length > 1e-12) {
        normalized_axis = {axis.x / axis_length, axis.y / axis_length, axis.z / axis_length};
    }
    double half_angle = radians / 2.0;
    double sin_half_angle = std::sin(half_angle);
    double cos_half_angle = std::cos(half_angle);
    // Return quaternion in the form (w, x, y, z)
    return {cos_half_angle, normalized_axis.x * sin_half_angle, normalized_axis.y * sin_half_angle, normalized_axis.z * sin_half_angle};
}

Vec3 rotate(Quat q, Vec3 v) {
    // TODO 1d: use q*(0,v)*conjugate(q), or an equivalent stable formula.
    // Convert vector to quaternion with w=0
    Quat v_quat{0, v.x, v.y, v.z};
    // Calculate the conjugate of q
    Quat q_conjugate{q.w, -q.x, -q.y, -q.z};
    // Perform the rotation: q * v_quat * q_conjugate
    Quat temp = multiply(q, v_quat);
    Quat rotated_quat = multiply(temp, q_conjugate);
    // Extract the rotated vector from the resulting quaternion
    v.x = rotated_quat.x;
    v.y = rotated_quat.y;
    v.z = rotated_quat.z;
    return v;
}
