#pragma once
#include "motion.hpp"
#include <array>
#include <vector>

using FootPair = std::array<Vec3, 2>;

struct Contact {
    bool planted = false; // false during swing
    Vec3 anchor;           // fixed target world-space goal during this stance
};

using ContactPair = std::array<Contact, 2>;

struct FootLockResult {
    std::vector<FootPair> goals;      // one hard-lock goal pair per frame
    std::vector<ContactPair> contacts; // one contact pair per frame
};

// Bonus 1: sourceWorld[f] holds source FK; movingGoals[f] holds the two
// target-space IK goals before any lock. sourceFeet are left/right joint indices,
// and frameTime is in seconds. Feet moving at 0.15 m/s or slower are planted;
// each planted interval keeps its first moving goal as the anchor.
// Return arrays matching movingGoals.size().
FootLockResult applyStanceAwareFootLock(
    const std::vector<WorldPose>& sourceWorld,
    const std::vector<FootPair>& movingGoals,
    std::array<int, 2> sourceFeet,
    double frameTime);

// Bonus 2: return one goal pair per frame. Keep swing goals unchanged and
// smoothly blend planted goals toward their anchors over up to two frames at
// each stance boundary.
std::vector<FootPair> applySmoothContactTransitions(
    const std::vector<FootPair>& movingGoals,
    const std::vector<ContactPair>& contacts);
