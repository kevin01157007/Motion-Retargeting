# CCD inverse kinematics for HW2

Implement `solveCCD` in `src/ik.cpp`. Its declaration is in `src/motion.hpp`:

```cpp
void solveCCD(const Skeleton& skeleton, Pose& pose, int endEffector,
              Vec3 target, int iterations);
```

The function changes `pose.localRotation` so the joint at `endEffector` moves toward `target`. The target is a **world-space position in meters**. The function does not move the root or change any bone offsets. It is called separately for the left and right target feet.

## What CCD does

Cyclic coordinate descent (CCD) adjusts one joint at a time. For a foot, start at its parent (the lower-leg joint), then move to that joint's parent (the upper-leg joint). Each adjustment rotates the current foot direction toward the desired foot direction around that joint. Repeat these two adjustments for up to `iterations` passes, stopping early if the foot is close enough.

For this assignment, adjust only those two leg joints. Do not rotate the foot joint itself or the skeleton root.

## One joint update

Call `forwardKinematics(skeleton, pose)` to get the current `WorldPose`. For the joint being adjusted, define:

- `pivot`: the joint's world-space position;
- `foot`: the end effector's world-space position;
- `current = foot - pivot`;
- `desired = target - pivot`.

Both vectors start at the same pivot. Find a quaternion `worldDelta` that rotates the direction of `current` toward `desired` in **world space**. The rotation axis comes from their cross product, and the angle comes from their dot product. Normalize the direction vectors first. If you use `acos`, clamp the dot product to `[-1, 1]` before calling it.

`Pose::localRotation[j]` is relative to joint `j`'s parent, so a world-space correction cannot be multiplied into it directly. Let `parentWorld` be the parent's world rotation from FK. For unit quaternions, its inverse is its conjugate. Convert the correction into the joint's local frame:

```text
localDelta = inverse(parentWorld) * worldDelta * parentWorld
newLocalRotation = normalize(localDelta * oldLocalRotation)
```

Quaternion multiplication order matters: `a * b` applies `b` first. Recompute FK after each joint update, because the foot position and world rotations have changed.

## Iteration order

```text
repeat for at most iterations passes:
    compute FK
    if distance(foot, target) < 0.01 m: stop
    for joint in [foot parent, that parent's parent]:
        compute FK for the latest pose
        calculate current and desired vectors around joint
        calculate a world-space alignment rotation
        convert it to a local rotation and update pose.localRotation[joint]
        compute FK again
        if distance(foot, target) < 0.01 m: stop
```

The `0.01 m` threshold is 1 cm. The `iterations` argument is the maximum number of passes, not the number of individual joint rotations.
