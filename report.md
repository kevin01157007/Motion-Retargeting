
# HW2: Motion Retargeting

## Method & formulas

This project transfers a captured walking motion from the source skeleton A to a different target skeleton B while preserving the original gait pattern and foot placement. The pipeline follows the assignment specification: quaternion math, forward kinematics (FK), inverse kinematics (CCD), and retargeting by joint-name mapping.

### 1. Quaternion math

Each joint stores a local rotation quaternion in the form $(w, x, y, z)$. Quaternion operations are implemented using the standard Hamilton product:

- Normalize: $q = \dfrac{q}{\lVert q \rVert}$
- Multiply: $q_a q_b = (w, \mathbf{v})$ with
    - $w = w_a w_b - \mathbf{v}_a \cdot \mathbf{v}_b$
    - $\mathbf{v} = w_a \mathbf{v}_b + w_b \mathbf{v}_a + \mathbf{v}_a \times \mathbf{v}_b$
- Axis-angle rotation:
    - $q = \left(\cos\dfrac{\theta}{2},\ \mathbf{n}\sin\dfrac{\theta}{2}\right)$
    - with $\mathbf{n} = \dfrac{\mathbf{axis}}{\lVert \mathbf{axis} \rVert}$
- Vector rotation: $\mathbf{v}' = q\,\mathbf{v}\,q^{-1}$

The composition convention is important: $a * b$ applies $b$ first and then $a$. The identity quaternion is $(1, 0, 0, 0)$.

### 2. Forward kinematics

The skeleton is a hierarchy stored in parent-before-child order. For a root joint:

- $P_0 = \text{rootTranslation} + \text{offset}_0$
- $Q_0 = \operatorname{normalize}(\text{localRotation}_0)$

For any child joint $j$ with parent $p$:

- $P_j = P_p + \operatorname{rotate}(Q_p,\ \text{offset}_j)$
- $Q_j = \operatorname{normalize}(Q_p \cdot \text{localRotation}_j)$

This ensures each bone is placed in world space using the parent transform and then oriented by the parent rotation followed by the local joint rotation.

### 3. CCD inverse kinematics

The left and right feet are solved independently with cyclic coordinate descent. For a foot target $\mathbf{t}$ in world coordinates, we choose the leg chain from the lower-leg parent to the upper-leg parent and iterate until the end effector reaches the target within $1\,\text{cm}$ or a maximum of $30$ iterations.

At each joint update, let:

- $\mathbf{pivot}$ = world position of the current joint
- $\mathbf{foot}$ = world position of the end effector
- $\mathbf{current} = \mathbf{foot} - \mathbf{pivot}$
- $\mathbf{desired} = \mathbf{t} - \mathbf{pivot}$

The alignment rotation is computed in world space using the cross and dot products:

- $\mathbf{axis} = \operatorname{normalize}(\mathbf{current} \times \mathbf{desired})$
- $\text{angle} = \arccos\Big(\operatorname{clamp}\big(\operatorname{normalize}(\mathbf{current}) \cdot \operatorname{normalize}(\mathbf{desired}),\ -1,\ 1\big)\Big)$

The world correction is then converted to the local frame of the adjusted joint:

- $\text{localDelta} = \text{parentWorld}^{-1} \cdot \text{worldDelta} \cdot \text{parentWorld}$
- $\text{newLocalRotation} = \operatorname{normalize}(\text{localDelta} \cdot \text{oldLocalRotation})$

Recomputing FK after each joint update keeps the chain consistent and allows the foot to converge toward the target while respecting the hierarchy.

### 4. Retargeting

The target rig uses the same joint hierarchy but with naming prefixed by B_, and the limb lengths differ from the source rig. Retargeting therefore does not assume matching indices. Instead, each target joint B_<name> is mapped to the source joint <name> by name; the source local rotation is copied and normalized into the target pose. The root translation is scaled by 1.12 to account for the target body proportions, while each target joint retains its own bone offset so the new limb lengths are respected. Unmatched names throw a clear error so the pipeline fails early instead of silently producing wrong motion.

## Implementation decisions

Several design choices were necessary to make the motion transfer robust and consistent with the assignment specification:

### 1. Quaternion math

### 2. Forward kinematics

### 3. CCD inverse kinematics

### 4. Retargeting


## FK/IK checks

The implementation was validated through the following checks:

- Quaternion checks: normalization produces unit length quaternions, Hamilton multiplication matches the expected composition order, and vector rotation yields the same direction after a known rotation.
- FK checks: a rotated parent should move its child in world space according to the parent rotation, and the root joint should remain at rootTranslation + offset[0].
- IK checks: both feet converge to their desired goal position within the 1 cm tolerance when the target is reachable. The solver is robust to zero-length or nearly collinear vectors and remains finite for unreachable cases.
- Retarget checks: each B_<name> joint resolves to a valid source joint, the root translation is scaled by 1.12, and the target motion maintains the gait timing without relying on a fixed index mapping.

In short, the FK results match the hierarchical transformation equations, and the CCD solver reaches each planted foot target while keeping the update local to the leg chain.

## Comparisons

The source and target rigs differ in both bone lengths and overall body scale. The source motion is a CMU walk captured from subject 02, trial 01; the target rig B is created specifically for the assignment and has the same parent structure but different limb and torso proportions. Therefore, the retargeting process is not a direct copy of the source pose but a semantic transfer of local rotations onto the target skeleton.

Compared with a naive same-index copy, the name-based mapping keeps the motion consistent with the rig's joint semantics. Because B retains its own offsets, the transferred motion appears as the same walking cycle but adjusted to the target proportions. The root translation also changes by the scaling factor 1.12, producing a larger overall target body while preserving the original stride rhythm and foot contact timing.

The viewer result should show a valid gait cycle with the same general movement pattern but with lengthened or shortened limbs depending on the target rig. The differences are most visible in the leg segments and torso spacing, while the forward motion and periodic foot placement remain recognizably from the source clip.

## Limitations

The current method is effective for a simple motion retargeting task, but it has several limitations:

- The method does not enforce joint-angle limits, so the regenerated motion can overshoot or violate anatomical constraints.
- CCD only solves the feet using a two-joint leg chain; it does not consider full-body balance, pelvis drift, or global support constraints.
- Retargeting preserves joint rotations but cannot perfectly account for differences in bone lengths and the center of mass without a full inverse-kinematic or optimization-based correction.
- The solver is local and may converge to a poor local minimum when the goal is unreachable or heavily angled.
- The assignment version uses the provided source and target rigs only; broader character retargeting would require more complete rig correspondence and contact handling.

These limitations are acceptable for the homework setting, but they also explain why a more production-ready retargeting system would require stronger constraints, better foot-locking logic, and more advanced motion optimization.