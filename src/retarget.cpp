#include "motion.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>

Pose retargetFrame(const Skeleton& source, const Skeleton& target,
                   const Pose& sourcePose, double rootScale) {
    // TODO 4: map B_<source-name> to each target joint, transfer local
    // quaternion motion, and normalize. Do not use same-index assumptions.
    // Report unmatched joints clearly. Distinct offsets in target.rig supply
    // different bone lengths. Root translation is scaled by rootScale.
    Pose result;
    result.rootTranslation = sourcePose.rootTranslation * rootScale;
    result.localRotation.resize(target.joints.size());

    std::unordered_map<std::string, std::size_t> sourceJointIndices;
    for (std::size_t i = 0; i < source.joints.size(); ++i) {
        sourceJointIndices[source.joints[i].name] = i;
    }

    for (std::size_t i = 0; i < target.joints.size(); ++i) {
        const std::string& targetName = target.joints[i].name;
        if (targetName.rfind("B_", 0) != 0) {
            throw std::runtime_error(
                "Target joint '" + targetName + "' must start with 'B_' to map to a source joint");
        }

        const std::string sourceName = targetName.substr(2);
        const auto sourceJoint = sourceJointIndices.find(sourceName);
        if (sourceJoint == sourceJointIndices.end()) {
            throw std::runtime_error(
                "No source joint '" + sourceName + "' matches target joint '" + targetName + "'");
        }
        if (sourceJoint->second >= sourcePose.localRotation.size()) {
            throw std::runtime_error(
                "Source pose has no rotation for joint '" + sourceName +
                "' required by target joint '" + targetName + "'");
        }

        result.localRotation[i] = normalize(sourcePose.localRotation[sourceJoint->second]);
    }

    return result;
}
