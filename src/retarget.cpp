#include "motion.hpp"

Pose retargetFrame(const Skeleton& source, const Skeleton& target,
                   const Pose& sourcePose, double rootScale) {
    Pose result;
    result.rootTranslation = sourcePose.rootTranslation * rootScale;
    result.localRotation.resize(target.joints.size());
    // TODO 4: map B_<source-name> to each target joint, transfer local
    // quaternion motion, and normalize. Do not use same-index assumptions.
    // Report unmatched joints clearly. Distinct offsets in target.rig supply
    // different bone lengths. Root translation is scaled by rootScale.
    (void)source;
    return result;
}
