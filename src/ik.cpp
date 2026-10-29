#include "motion.hpp"

void solveCCD(const Skeleton& skeleton, Pose& pose, int endEffector,
              Vec3 target, int iterations) {
    // TODO 3: cyclic coordinate descent for the two leg joints above the foot.
    // Recompute FK after each joint update. Stop at the hip/upper-leg joint;
    // do not rotate the root. Use clamped dot products and handle zero vectors.
    // The target may be unreachable; minimize distance without NaNs.
    (void)skeleton; (void)pose; (void)endEffector; (void)target; (void)iterations;
}
