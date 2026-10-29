# HW2 - Motion Retargeting

Read [hw2.pdf](hw2.pdf) before editing. Use [MOTION_TYPES.md](MOTION_TYPES.md) as a reference for the types in `src/motion.hpp` and [CCD_IK.md](CCD_IK.md) for the IK task. Implement the required TODOs in `src/math.cpp`, `src/fk.cpp`, `src/ik.cpp`, and `src/retarget.cpp`; the two optional bonus TODOs are isolated in `src/bonus.cpp`. The starter builds and exports a viewer, but its unimplemented functions show rest poses rather than a correct animation. The viewer includes bonus checkboxes; their modes remain identical until you implement contact detection, foot locking, and blending.

## Build

From this directory:

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

A C++17 compiler is required. CMake uses the system compiler when available. On Linux/macOS machines without one, it can use the optional Python `ziglang` compiler package (`python3 -m pip install --user ziglang`). If configuration previously failed, rerun the same CMake command; the fallback repairs the failed `build` cache.

Run on Linux or macOS:

```sh
./build/hw2 data/source.motion data/target.rig build/viewer.html
```

Run on Windows with a multi-config generator (Visual Studio):

```powershell
.\build\Release\hw2.exe data\source.motion data\target.rig build\viewer.html
```

With a single-config Windows generator, use `build\hw2.exe`. Open `viewer.html` in a browser. Add `--frames 120` to export a shorter preview.

## Starter layout

- `data/cmu_02_01.bvh`: source CMU walking motion, a community BVH conversion.
- `data/source.motion`: prepared A rig and sampled motion.
- `data/target.rig`: B rig with different limb proportions and `B_` joint names.
- `tools/prepare_data.py`: reproducible data preparation; Python 3 is only needed to rerun it.
- `src/main.cpp`: data loading, per-frame pipeline, and a self-contained HTML viewer.
- `src/bonus.hpp` and `src/bonus.cpp`: two optional functions for stance-aware foot lock and smooth transitions.

The source data has 31 joints, 172 sampled frames, Y-up coordinates, and meter-scale positions. The BVH values are multiplied by 0.0254 during preparation. The source BVH has 344 frames at 120 Hz; every second frame is used for this exercise. Skeleton joints are ordered with the parent before its children. Each frame has one root translation plus one local quaternion per joint.

## Attribution

The data used in this project was obtained from [CMU Graphics Lab Motion Capture Database](https://mocap.cs.cmu.edu/). CMU lists subject 02, trial 01 as a walk. The bundled BVH comes from the [CMU BVH conversion mirror](https://github.com/una-dinosauria/cmu-mocap/blob/master/data/002/02_01.bvh). The generated B rig is specific to this assignment.
