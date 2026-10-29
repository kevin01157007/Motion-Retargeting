#include "bonus.hpp"

FootLockResult applyStanceAwareFootLock(
    const std::vector<WorldPose>& sourceWorld,
    const std::vector<FootPair>& movingGoals,
    std::array<int, 2> sourceFeet,
    double frameTime) {
    // TODO bonus 1: detect stance for each foot, save a world-space anchor,
    // and use that anchor as the IK goal until the foot leaves contact.
    (void)sourceWorld;
    (void)sourceFeet;
    (void)frameTime;
    return {movingGoals, std::vector<ContactPair>(movingGoals.size())};
}

std::vector<FootPair> applySmoothContactTransitions(
    const std::vector<FootPair>& movingGoals,
    const std::vector<ContactPair>& contacts) {
    // TODO bonus 2: blend moving IK goals toward planted anchors near each
    // contact boundary, using a smooth weight that reaches 1 in mid-stance.
    (void)contacts;
    return movingGoals;
}
