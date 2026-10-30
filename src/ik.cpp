#include "motion.hpp"

#include <algorithm>
#include <cmath>

namespace {

Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}

Vec3 anyPerpendicular(Vec3 v) {            // v 已正規化
    const Vec3 basis = std::abs(v.x) <= std::abs(v.y) && std::abs(v.x) <= std::abs(v.z)
        ? Vec3{1,0,0} : (std::abs(v.y) <= std::abs(v.z) ? Vec3{0,1,0} : Vec3{0,0,1});
    return cross(v, basis);
}

void rotateJointToward(const Skeleton& sk, Pose& pose, int joint, int effector, Vec3 target) {
    const WorldPose w = forwardKinematics(sk, pose);
    const Vec3 pivot = w.position[joint];
    Vec3 cur = w.position[effector] - pivot;
    Vec3 des = target - pivot;
    const double cl = length(cur), dl = length(des);
    if (!(cl > 1e-9 && dl > 1e-9)) return;   // 同時擋掉 NaN 與零向量
    cur = cur * (1.0 / cl);
    des = des * (1.0 / dl);

    const double c = std::clamp(dot(cur, des), -1.0, 1.0);
    Vec3 axis = cross(cur, des);
    if (length(axis) < 1e-12) {
        if (c >= 0.0) return;                // 已對齊
        axis = anyPerpendicular(cur);        // 反向
    }

    const Quat worldDelta = axisAngle(axis, std::acos(c));   // axisAngle 會自己正規化軸
    const Quat pw = w.rotation[sk.joints[joint].parent];     // FK 已 normalize
    const Quat inv{pw.w, -pw.x, -pw.y, -pw.z};
    const Quat localDelta = multiply(multiply(inv, worldDelta), pw);
    pose.localRotation[joint] = normalize(multiply(localDelta, pose.localRotation[joint]));
}
} // namespace

void solveCCD(const Skeleton& sk, Pose& pose, int ee, Vec3 target, int iterations) {
    const int n = static_cast<int>(sk.joints.size());
    if (iterations <= 0 || ee < 0 || ee >= n || pose.localRotation.size() != sk.joints.size()) return;

    const int j1 = sk.joints[ee].parent;
    if (j1 < 0) return;
    const int chain[] = {j1, sk.joints[j1].parent};
    constexpr double tol = 0.01;

    for (int it = 0; it < iterations; ++it) {
        for (int joint : chain) {
            if (length(target - forwardKinematics(sk, pose).position[ee]) < tol) return;
            if (joint < 0 || sk.joints[joint].parent < 0) continue;   // 不轉 root
            rotateJointToward(sk, pose, joint, ee, target);
        }
        if (length(target - forwardKinematics(sk, pose).position[ee]) < tol) return;
    }
}
