#include "bonus.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr double kStanceSpeedThreshold = 0.15; // meters per second

double smoothstep(double value) {
    const double t = std::clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}

FootLockResult applyStanceAwareFootLock(
    const std::vector<WorldPose>& sourceWorld,
    const std::vector<FootPair>& movingGoals,
    std::array<int, 2> sourceFeet,
    double frameTime) {
    if (sourceWorld.size() != movingGoals.size()) {
        throw std::invalid_argument("sourceWorld and movingGoals must have the same frame count");
    }
    if (!std::isfinite(frameTime) || frameTime <= 0.0) {
        throw std::invalid_argument("frameTime must be finite and positive");
    }
    for (const int foot : sourceFeet) {
        if (foot < 0) {
            throw std::out_of_range("source foot indices must be nonnegative");
        }
        for (const WorldPose& pose : sourceWorld) {
            if (static_cast<size_t>(foot) >= pose.position.size()) {
                throw std::out_of_range("source foot index is outside a world pose");
            }
        }
    }

    FootLockResult result{movingGoals, std::vector<ContactPair>(movingGoals.size())};
    for (size_t frame = 0; frame < movingGoals.size(); ++frame) {
        for (size_t side = 0; side < sourceFeet.size(); ++side) {
            if (sourceWorld.size() < 2) {
                continue;
            }

            double speed = 0.0;
            if (frame == 0) {
                speed = length(sourceWorld[1].position[sourceFeet[side]] -
                               sourceWorld[0].position[sourceFeet[side]]) / frameTime;
            } else if (frame + 1 == sourceWorld.size()) {
                speed = length(sourceWorld[frame].position[sourceFeet[side]] -
                               sourceWorld[frame - 1].position[sourceFeet[side]]) / frameTime;
            } else {
                const double previousSpeed =
                    length(sourceWorld[frame].position[sourceFeet[side]] -
                           sourceWorld[frame - 1].position[sourceFeet[side]]) / frameTime;
                const double nextSpeed =
                    length(sourceWorld[frame + 1].position[sourceFeet[side]] -
                           sourceWorld[frame].position[sourceFeet[side]]) / frameTime;
                speed = std::max(previousSpeed, nextSpeed);
            }

            Contact& contact = result.contacts[frame][side];
            contact.planted = speed <= kStanceSpeedThreshold;
            if (contact.planted) {
                if (frame == 0 || !result.contacts[frame - 1][side].planted) {
                    contact.anchor = movingGoals[frame][side];
                } else {
                    contact.anchor = result.contacts[frame - 1][side].anchor;
                }
                result.goals[frame][side] = contact.anchor;
            }
        }
    }
    return result;
}

std::vector<FootPair> applySmoothContactTransitions(
    const std::vector<FootPair>& movingGoals,
    const std::vector<ContactPair>& contacts) {
    if (movingGoals.size() != contacts.size()) {
        throw std::invalid_argument("movingGoals and contacts must have the same frame count");
    }

    std::vector<FootPair> result = movingGoals;
    for (size_t side = 0; side < 2; ++side) {
        size_t frame = 0;
        while (frame < contacts.size()) {
            if (!contacts[frame][side].planted) {
                ++frame;
                continue;
            }

            const size_t start = frame;
            while (frame + 1 < contacts.size() && contacts[frame + 1][side].planted) {
                ++frame;
            }
            const size_t end = frame;
            const size_t segmentLength = end - start + 1;
            const double rampFrames =
                segmentLength <= 2 ? 1.0 : std::min(2.0, (segmentLength + 1) / 2.0);

            for (size_t plantedFrame = start; plantedFrame <= end; ++plantedFrame) {
                const size_t fromStart = plantedFrame - start + 1;
                const size_t toEnd = end - plantedFrame + 1;
                const double weight = smoothstep(
                    std::min(fromStart, toEnd) / rampFrames);
                const Vec3 anchor = contacts[plantedFrame][side].anchor;
                result[plantedFrame][side] =
                    movingGoals[plantedFrame][side] * (1.0 - weight) + anchor * weight;
            }
            ++frame;
        }
    }
    return result;
}
