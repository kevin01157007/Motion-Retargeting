#include "motion.hpp"

WorldPose forwardKinematics(const Skeleton& skeleton, const Pose& pose) {
    WorldPose world;
    world.position.resize(skeleton.joints.size());
    world.rotation.resize(skeleton.joints.size());
    // TODO 2: traverse joints in parent-before-child order. For a root,
    // position = rootTranslation + offset; rotation = localRotation.
    // For a child: position = parentPosition + rotate(parentRotation, offset)
    // and rotation = normalize(parentRotation * localRotation).
    // The lines below only show both rigs in their rest shape.
    for (size_t i = 0; i < skeleton.joints.size(); ++i) {
        const auto& j = skeleton.joints[i];
        world.position[i] = j.parent < 0 ? pose.rootTranslation + j.offset
                                         : world.position[j.parent] + rotate(world.rotation[j.parent], j.offset);
        world.rotation[i] = j.parent < 0 ? normalize(pose.localRotation[i])
                                         : normalize(multiply(world.rotation[j.parent], pose.localRotation[i]));
    }
    return world;
}