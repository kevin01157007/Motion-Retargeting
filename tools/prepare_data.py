"""Convert the bundled CMU BVH to the small, explicit HW2 text format.

Only the teaching data is generated here. All runtime FK, IK and retargeting
remain C++ student work. Run from any directory with Python 3.
"""
from pathlib import Path
import math
import re

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / "data/cmu_02_01.bvh"
tokens = re.findall(r"\{|\}|[^\s{}]+", source.read_text())
at = 0
joints = []


def take(expected=None):
    global at
    value = tokens[at]
    at += 1
    if expected is not None and value != expected:
        raise ValueError(f"expected {expected}, got {value}")
    return value


def parse_joint(parent, kind):
    take(kind)
    name = take()
    take("{")
    take("OFFSET")
    offset = tuple(float(take()) * .0254 for _ in range(3))
    take("CHANNELS")
    channels = [take() for _ in range(int(take()))]
    index = len(joints)
    joints.append((name, parent, offset, channels))
    while tokens[at] != "}":
        if tokens[at] == "JOINT":
            parse_joint(index, "JOINT")
        elif tokens[at] == "End":
            take("End")
            take("Site")
            take("{")
            take("OFFSET")
            for _ in range(3):
                take()
            take("}")
        else:
            raise ValueError(tokens[at])
    take("}")


def mul(a, b):
    w, x, y, z = a
    v, p, q, r = b
    return (w*v-x*p-y*q-z*r, w*p+x*v+y*r-z*q,
            w*q-x*r+y*v+z*p, w*r+x*q-y*p+z*v)


def axis_angle(axis, degrees):
    h = math.radians(degrees) / 2
    s = math.sin(h)
    return (math.cos(h), *(s if axis == a else 0. for a in "XYZ"))


take("HIERARCHY")
parse_joint(-1, "ROOT")
take("MOTION")
take("Frames:")
frames = int(take())
take("Frame")
take("Time:")
dt = float(take())
channel_count = sum(len(j[3]) for j in joints)
stride = 2
sampled = []
for f in range(frames):
    values = [float(take()) for _ in range(channel_count)]
    if f % stride:
        continue
    cursor = 0
    pose = []
    for name, parent, offset, channels in joints:
        q = (1., 0., 0., 0.)
        t = [0., 0., 0.]
        for channel in channels:
            value = values[cursor]
            cursor += 1
            if channel.endswith("rotation"):
                q = mul(q, axis_angle(channel[0], value))
            elif channel.endswith("position"):
                t["XYZ".index(channel[0])] = value * .0254
            else:
                raise ValueError(channel)
        pose.append((t, q))
    sampled.append(pose)

with (ROOT / "data/source.motion").open("w") as out:
    out.write(f"HW2MOTION 1\n{len(joints)} {len(sampled)} {dt*stride:.7f}\n")
    for name, parent, offset, _ in joints:
        out.write(f"{name} {parent} {' '.join(f'{v:.7f}' for v in offset)}\n")
    for pose in sampled:
        for t, q in pose:
            out.write(" ".join(f"{v:.8f}" for v in (*t, *q)) + "\n")

with (ROOT / "data/target.rig").open("w") as out:
    out.write(f"HW2RIG 1\n{len(joints)}\n")
    for name, parent, offset, _ in joints:
        if "Leg" in name or "Foot" in name or "Toe" in name:
            scale = (1.12, 1.20, 1.12)
        elif "Arm" in name or "Shoulder" in name or "Hand" in name or "Thumb" in name or "Finger" in name:
            scale = (0.83, 0.88, 0.83)
        else:
            scale = (1.10, 1.08, 1.10)
        scaled = [a*b for a, b in zip(offset, scale)]
        out.write(f"B_{name} {parent} {' '.join(f'{v:.7f}' for v in scaled)}\n")

print(f"Converted {frames} BVH frames to {len(sampled)} frames, {len(joints)} joints")
