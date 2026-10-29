#include "motion.hpp"

Quat normalize(Quat q) {
    // TODO 1a: return q divided by its length. Handle a near-zero quaternion.
    return q;
}

Quat multiply(Quat a, Quat b) {
    // TODO 1b: Hamilton product. Composition convention: a*b applies b, then a.
    (void)b;
    return a;
}

Quat axisAngle(Vec3 axis, double radians) {
    // TODO 1c: normalize axis, use cos(theta/2) and sin(theta/2).
    (void)axis; (void)radians;
    return {};
}

Vec3 rotate(Quat q, Vec3 v) {
    // TODO 1d: use q*(0,v)*conjugate(q), or an equivalent stable formula.
    (void)q;
    return v;
}
