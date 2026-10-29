# `motion.hpp` reference

The header defines seven C++ `struct` types. They describe a skeleton, one pose, a sequence of poses, and the math values used by the animation functions. Array indices refer to the same joint across `Skeleton::joints`, `Pose::localRotation`, and `WorldPose`.

Coordinates use meters and Y-up. Angles passed to `axisAngle` use radians. Source and target skeletons may have different bone lengths.

## `Vec3`

A three-dimensional vector with `double x`, `y`, and `z`, all initially zero. It represents positions, translations, offsets, axes, and IK goals. The header provides vector addition, subtraction, multiplication by a scalar on the right (`vector * scalar`), `dot`, and `length`.

The same type holds both local and world values. Its meaning depends on where it is stored: `Joint::offset` is local to the parent joint, while `WorldPose::position` and the `solveCCD` target are world-space values.

## `Quat`

A quaternion with components `double w`, `x`, `y`, and `z`. Its default value `(1, 0, 0, 0)` is the identity rotation. Local joint rotations and computed world rotations both use this type.

`normalize` returns a unit quaternion; `multiply(a, b)` composes rotations so `b` acts first; `axisAngle(axis, radians)` creates a rotation; and `rotate(q, v)` applies a quaternion to a vector. These functions are declared in the header and implemented in `src/math.cpp`.

## `Joint`

One node in a skeleton:

| Field | Meaning |
| --- | --- |
| `name` | Joint identifier, such as `LeftFoot` or `B_LeftFoot`. |
| `parent` | Index of the parent in `Skeleton::joints`; `-1` marks the root. |
| `offset` | Rest offset from the parent, expressed in the parent's local coordinates. |

The root also has an offset. Its world position is `Pose::rootTranslation + root.offset`.

## `Skeleton`

`joints` is the ordered list of `Joint` values that defines the hierarchy and bone lengths. A parent appears before its children, so a joint's nonnegative `parent` index is smaller than its own index. The source A skeleton and target B skeleton have separate offsets. Target names have a `B_` prefix for retargeting.

## `Pose`

One animation frame in local coordinates:

| Field | Meaning |
| --- | --- |
| `rootTranslation` | Translation of the skeleton root in world coordinates, before adding the root joint's offset. |
| `localRotation` | One quaternion per joint, indexed exactly like `Skeleton::joints`. Each child rotation is relative to its parent. |

`Pose` does not contain final joint positions. Forward kinematics computes those positions using a `Skeleton` and this pose.

## `Motion`

A sampled animation clip:

| Field | Meaning |
| --- | --- |
| `skeleton` | The skeleton shared by all frames in the clip. |
| `frames` | Time-ordered `Pose` values. |
| `frameTime` | Time between consecutive frames, in seconds. |

Frame `i` occurs at approximately `i * frameTime` seconds. In the supplied clip, `Motion::skeleton` is source A; target B is loaded separately as a `Skeleton`.

## `WorldPose`

The result of forward kinematics for one frame. `position[i]` is joint `i`'s world-space position, and `rotation[i]` is its world-space orientation. Both vectors have one entry per skeleton joint. Use `WorldPose` to inspect animated joint locations, including the current positions of the feet.

## Functions declared after the types

| Function | Input and result |
| --- | --- |
| `forwardKinematics(skeleton, pose)` | Returns a `WorldPose` for the supplied local pose. The pose must have one local rotation per skeleton joint. |
| `solveCCD(skeleton, pose, endEffector, target, iterations)` | Modifies `pose` to move a chosen joint toward a world-space `target`, using up to `iterations` CCD passes. The target may be unreachable. |
| `retargetFrame(source, target, sourcePose, rootScale)` | Returns a target `Pose`. It matches target `B_` joint names to source names, transfers local rotations, and scales the root translation. Target bone lengths come from the target `Skeleton` when FK is run. |

The per-frame flow is `source Pose` -> source FK -> retargeted target `Pose` -> optional foot IK -> target FK -> `WorldPose` for display.
